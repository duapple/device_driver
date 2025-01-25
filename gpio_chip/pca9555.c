#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/gpio.h>

#define GPIO_NUM 16
#define GPIO_PORT_NUM 2

struct gpio_port {
    u8 val;
    u8 polarity;
    u8 direction;
};

struct pca9555_dev {
    struct i2c_client *client;
    struct gpio_chip chip;

    struct gpio_port port[GPIO_PORT_NUM];
};

static int pca9555_i2c_read_bytes(struct i2c_client *client, u8 cmd, u8 *buf,
                                u8 size)
{
    return 0;
}

static int pca9555_i2c_write_bytes(struct i2c_client *client, u8 cmd, u8 *buf,
                                u8 size)
{
    return 0;
}

static int pca9555_direction_input(struct gpio_chip *chip, unsigned offset)
{

    return 0;
}

static int pca9555_direction_output(struct gpio_chip *chip,
                                    unsigned offset, int value)
{
    return 0;
}

static int pca9555_get_value(struct gpio_chip *chip, unsigned offset)
{
    return 0;
}

static void pca9555_set_value(struct gpio_chip *chip, unsigned offset,
                            int value)
{
}

static int pca9555_init(struct pca9555_dev *pca9555)
{

}

static int pca9555_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    struct pca9555_dev *pca9555 = NULL;
    int ret = 0;
    struct gpio_chip *chip = NULL;

    pca9555 = devm_kzalloc(&client->dev, sizeof(*pca9555), GFP_KERNEL);
    if (IS_ERR(pca9555)) {
        return PTR_ERR(pca9555);
    }

    i2c_set_clientdata(client, pca9555);
    pca9555->client = client;
    chip = &pca9555->chip;

    chip->label = client->name;
    chip->base = -1;
    chip->parent = &client->dev;
    chip->owner = THIS_MODULE;
    chip->ngpio = GPIO_NUM;
    chip->can_sleep = 1;
    chip->get = pca9555_get_value;
    chip->set = pca9555_set_value;
    chip->direction_input = pca9555_direction_input;
    chip->direction_output = pca9555_direction_output;

    ret = devm_gpiochip_add_data(&client->dev, chip, NULL);
    if (ret) {
        dev_err(&client->dev, "gpiochip add error: %d", -ret);
        return ret;
    }

    return 0;
}

static int pca9555_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id pca9555_id_table[] = {
    {"pca9555", 0},
    { },
};
MODULE_DEVICE_TABLE(i2c, pca9555_id_table);

static const struct of_device_id pca9555_of_match[] = {
    {.compatible = "nxp,pca9555"},
    { },
};
MODULE_DEVICE_TABLE(of, pca9555_of_match);

static struct i2c_driver pca9555_driver = {
    .probe = pca9555_probe,
    .remove = pca9555_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "pca9555_drv",
        .of_match_table = of_match_ptr(pca9555_of_match),
    },
    .id_table = pca9555_id_table,
};

module_i2c_driver(pca9555_driver);

MODULE_AUTHOR("duapple <duapple2@gmail.com>");
MODULE_LICENSE("GPL");