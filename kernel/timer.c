#include "linux/fs.h"
#include "linux/interrupt.h"
#include "linux/jiffies.h"
#include "linux/mutex.h"
#include "linux/of.h"
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>

struct timer_dev {
    struct platform_device *pdev;
    struct timer_list timer;
    struct cdev cdev;
    dev_t dev_id;
    struct class *class;
    struct device *device;

    wait_queue_head_t wq;
    struct mutex lock;
    char buf[128];
    u8 ready;

    struct gpio_desc *intb;
    int irq;
};

static int timer_cdev_open(struct inode *inode, struct file *filp)
{
    struct timer_dev *tim = container_of(inode->i_cdev,
                                        struct timer_dev, cdev);

    filp->private_data = tim;
    return 0;
}

static ssize_t timer_cdev_read(struct file *filp, char __user *buf, size_t size,
                                loff_t *loft)
{
    struct timer_dev *tim = filp->private_data;
    size_t len = 5; // ARRAY_SIZE(tim->buf) < size ? ARRAY_SIZE(tim->buf) : size;

    /* Notice!!! In the mutex period, don't do time consuming work*/
    if (wait_event_timeout(tim->wq, tim->ready, msecs_to_jiffies(700)) <= 0) {
        return -EBUSY;
    }

    if (mutex_lock_interruptible(&tim->lock)) {
        return -ERESTARTSYS;
    }

    if (copy_to_user(buf, tim->buf, len)) {
        mutex_unlock(&tim->lock);
        dev_err(&tim->pdev->dev, "copy_to_user error: 0x%x", -EIO);
        return -EIO;
    }

    memset(tim->buf, 0, ARRAY_SIZE(tim->buf));
    tim->ready = 0;
    mutex_unlock(&tim->lock);
    return len;
}

static struct file_operations timer_cdev_fops = {
    .open = timer_cdev_open,
    .read = timer_cdev_read,
};

static void timer_cb(struct timer_list *timer)
{
    struct timer_dev *tim = container_of(timer, struct timer_dev, timer);

    mod_timer(&tim->timer, jiffies + msecs_to_jiffies(1000));

    dev_info(&tim->pdev->dev, "timer_cb invoked");

    mutex_lock(&tim->lock);
    memset(tim->buf, 'A', ARRAY_SIZE(tim->buf));
    tim->ready = 1;
    wake_up_interruptible(&tim->wq);
    mutex_unlock(&tim->lock);
}

static irqreturn_t intb_irq_handler(int irq, void *data)
{
    pr_err("intb irq handler\n");
    return IRQ_HANDLED;
}

static int timer_probe(struct platform_device *pdev)
{
    struct timer_dev *tim;
    int ret = 0;

    tim = devm_kzalloc(&pdev->dev, sizeof(struct timer_dev), GFP_KERNEL);

    tim->pdev = pdev;
    tim->ready = 0;
    platform_set_drvdata(pdev, tim);

    timer_setup(&tim->timer, timer_cb, 0);
    // mod_timer(&tim->timer, jiffies + msecs_to_jiffies(1000));

    ret = alloc_chrdev_region(&tim->dev_id, 0, 1, "timer_cdev");
    if (ret) {
        return -ENODEV;
    }

    cdev_init(&tim->cdev, &timer_cdev_fops);
    
    ret = cdev_add(&tim->cdev, tim->dev_id, 1);
    if (ret) {
        goto err1;
    }

    tim->class = class_create(THIS_MODULE, "timer_cdev_class");
    if (IS_ERR(tim->class)) {
        ret = PTR_ERR(tim->class);
        goto err2;
    }

    tim->device = device_create(tim->class, &pdev->dev, tim->dev_id, NULL, "timer_dev");
    if (IS_ERR(tim->device)) {
        ret = PTR_ERR(tim->device);
        goto err3;
    }

    init_waitqueue_head(&tim->wq);
    mutex_init(&tim->lock);

    tim->intb = devm_gpiod_get(&pdev->dev, "intb", GPIOD_IN);
    if (IS_ERR(tim->intb)) {
        ret = PTR_ERR(tim->intb);
        goto err4;
    }

    tim->irq = gpiod_to_irq(tim->intb);
    ret = devm_request_threaded_irq(&pdev->dev, tim->irq, NULL, intb_irq_handler,
        IRQF_ONESHOT | IRQF_TRIGGER_FALLING, "intb", tim);
    if (ret) {
        goto err4;
    }

    dev_info(&pdev->dev, "timer probe");
    return 0;
err4:
    device_destroy(tim->class, tim->dev_id);
err3:
    class_destroy(tim->class);
err2:
    cdev_del(&tim->cdev);
err1:
    unregister_chrdev_region(tim->dev_id, 1);
    return ret;
}

static int timer_remove(struct platform_device *pdev)
{
    struct timer_dev *tim = platform_get_drvdata(pdev);

    del_timer_sync(&tim->timer);
    mutex_destroy(&tim->lock);

    cdev_del(&tim->cdev);
    unregister_chrdev_region(tim->dev_id, 1);
    device_destroy(tim->class, tim->dev_id);
    class_destroy(tim->class);

    dev_info(&pdev->dev, "timer remove");
    return 0;
}

static struct of_device_id timer_of_match[] = {
    {.compatible = "owner,timer"},
    {},
};
MODULE_DEVICE_TABLE(of, timer_of_match);

static struct platform_driver timer_driver = {
    .probe = timer_probe,
    .remove = timer_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "timer_drv",
        .of_match_table = of_match_ptr(timer_of_match),
    }
};

module_platform_driver(timer_driver);

MODULE_AUTHOR("duapple <duapple2@gmail.com>");
MODULE_LICENSE("GPL");