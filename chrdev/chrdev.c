#include "linux/device.h"
#include "linux/fs.h"
#include "linux/platform_device.h"
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define OTA_DEV_NAME "ota_dev"

struct ota_dev {
    struct cdev cdev;
    dev_t devid;
    struct class *class;
    struct device *device;
    struct platform_device *pdev;
};

static int ota_dev_open(struct inode *inode, struct file *filp)
{
    struct ota_dev *ota = container_of(inode->i_cdev, struct ota_dev, cdev);
    filp->private_data = ota;
    return 0;
}

static ssize_t ota_dev_write(struct file *filp, const char __user *buf,
                            size_t size, loff_t *loft)
{
    return 0;
}

static ssize_t ota_dev_read(struct file *filp, char __user *buf, size_t size,
                            loff_t *loft)
{
    return 0;
}

static long ota_dev_ioctl(struct file *filp, u32 cmd, unsigned long arg)
{
    return 0;
}

static int ota_dev_close(struct inode *inode, struct file *filp)
{
    return 0;
}

static const struct file_operations ota_dev_fops = {
    .open = ota_dev_open,
    .write = ota_dev_write,
    .read = ota_dev_read,
    .compat_ioctl = ota_dev_ioctl,
    .release = ota_dev_close,
};

static int ota_dev_probe(struct platform_device *pdev)
{
    struct ota_dev *ota;
    struct device *dev = &pdev->dev;

    ota = devm_kzalloc(dev, sizeof(*ota), GFP_KERNEL);
    if (IS_ERR(ota)) {
        return PTR_ERR(ota);
    }

    platform_set_drvdata(pdev, ota);
    ota->pdev = pdev;

    alloc_chrdev_region(&ota->devid, 0, 1, OTA_DEV_NAME);
    cdev_init(&ota->cdev, &ota_dev_fops);
    cdev_add(&ota->cdev, ota->devid, 1);

    ota->class = class_create(THIS_MODULE, OTA_DEV_NAME);

    ota->device = device_create(ota->class, dev, ota->devid, NULL, OTA_DEV_NAME);

    dev_info(dev, "ota_dev probe success");
    return 0;
}

static int ota_dev_remove(struct platform_device *pdev)
{
    struct ota_dev *ota = platform_get_drvdata(pdev);

    device_destroy(ota->class, ota->devid);
    class_destroy(ota->class);

    cdev_del(&ota->cdev);
    unregister_chrdev_region(ota->devid, 1);

    dev_info(&ota->pdev->dev, "ota_dev remove success");
    return 0;
}

static const struct of_device_id ota_dev_of_match[] = {
    {.compatible = "owner,ota_dev"},
    { },
};
MODULE_DEVICE_TABLE(of, ota_dev_of_match);

static struct platform_driver ota_dev_driver = {
    .probe = ota_dev_probe,
    .remove = ota_dev_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "ota_driver",
        .of_match_table = of_match_ptr(ota_dev_of_match),
    },
};

module_platform_driver(ota_dev_driver);

MODULE_AUTHOR("duapple <duapple2@gmail.com>");
MODULE_LICENSE("GPL");