#include "asm-generic/errno-base.h"
#include "linux/device.h"
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

struct htu21d_dev {
    struct i2c_client *client;

    s32 temperature;
    u32 humidity;
};

/* x^8 + x^5 + x^4 + 1: 100110001 (00110001 -> 0x31) */
static u8 crc8_calcu(u8 *data, size_t len)
{
    /* init value: 0x0 */
    u8 crc = 0x0;
    int i, j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
        }
    }
    return crc;
}

static int htu21d_i2c_read_bytes(struct i2c_client *client, u16 chip, u8 reg,
                                u8 *data, u8 size)
{
    struct i2c_msg msg[] = {
        {
            .flags = 0,
            .addr = client->addr,
            .buf = &reg,
            .len = 1,
        },
        {
            .flags = I2C_M_RD | I2C_M_STOP,
            .addr = client->addr,
            .buf = data,
            .len = size,
        },
    };

    if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
        dev_err(&client->dev, "i2c read error, chip=0x%x, reg=0x%x\n",
                client->addr, reg);
        return -EREMOTEIO;
    }

    return 0;
}

static int htu21d_reset(struct htu21d_dev *htu21d)
{
    struct i2c_client *client = htu21d->client;
    u8 val = 0xFE;
    struct i2c_msg msg[] = {
        {
            .flags = I2C_M_STOP,
            .addr = client->addr,
            .buf = &val,
            .len = 1,
        },
    };
    int ret;

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    if (ret != ARRAY_SIZE(msg)) {
        dev_err(&client->dev, "i2c write reg error, chip=0x%02x, reg=0x%02x, ret=%d\n",
                client->addr, val, ret);
        return -EREMOTEIO;
    }

    /* soft reset take at least 15ms */
    msleep(15);

    return 0;
}

static int htu21d_config(struct htu21d_dev *htu21d)
{
    struct i2c_client *client = htu21d->client;
    u8 data[] = {0xE6, 0x01};
    struct i2c_msg msg[] = {
        {
            .flags = I2C_M_STOP,
            .addr = client->addr,
            .buf = (unsigned char *)&data,
            .len = ARRAY_SIZE(data),
        },
    };
    int ret = 0;

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    if (ret != ARRAY_SIZE(msg)) {
        dev_err(&client->dev, "i2c write reg error, chip=0x%02x, reg=0x%02x, ret=%d\n",
                client->addr, data[0], ret);
        return -EREMOTEIO;
    }

    return 0;
}

/* actual value T = temp / 100 */
static int htu21d_read_temperature(struct htu21d_dev *htu21d, s32 *temp)
{
    u8 data[3] = {0};
    struct i2c_client *client = htu21d->client;
    int ret = 0;
    u32 s_temp;
    u8 crc;

    *temp = 0;

    /* hold master */
    ret = htu21d_i2c_read_bytes(client, client->addr, 0xE3, data, ARRAY_SIZE(data));
    if (ret) {
        return ret;
    }

    crc = crc8_calcu(data, 2);
    if (crc != data[2]) {
        dev_err(&client->dev, "temp data crc error\n");
        return -EINVAL;
    }

    s_temp = (data[0] << 8) | data[1];
    *temp = -4685 + 17572 * s_temp / (1 << 16);

    return 0;
}

/* actual value H = hum / 100 */
static int htu21d_read_humidity(struct htu21d_dev *htu21d, u32 *hum)
{
    u8 data[3] = {0};
    struct i2c_client *client = htu21d->client;
    int ret = 0;
    u32 s_rh;
    u8 crc;

    *hum = 0;

    /* hold master */
    ret = htu21d_i2c_read_bytes(client, client->addr, 0xE5, data, ARRAY_SIZE(data));
    if (ret) {
        return ret;
    }

    crc = crc8_calcu(data, 2);
    if (crc != data[2]) {
        dev_err(&client->dev, "humi data crc error\n");
        return -EINVAL;
    }

    s_rh = (data[0] << 8)| data[1];
    *hum = -600 + 12500 * s_rh / (1 << 16);

    return 0;
}

static ssize_t htu21d_temp_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct htu21d_dev *htu21d = i2c_get_clientdata(client);

    htu21d_read_temperature(htu21d, &htu21d->temperature);
    return sprintf(buf, "%d\n", htu21d->temperature);
}

static ssize_t htu21d_temp_store(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t count)
{
    return 0;
}
static DEVICE_ATTR(temperature, 0644, htu21d_temp_show, htu21d_temp_store);

static ssize_t htu21d_hum_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct htu21d_dev *htu21d = i2c_get_clientdata(client);

    htu21d_read_humidity(htu21d, &htu21d->humidity);
    return sprintf(buf, "%u\n", htu21d->humidity);
}

static ssize_t htu21d_hum_store(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t count)
{
    return 0;
}
static DEVICE_ATTR(humidity, 0644, htu21d_hum_show, htu21d_hum_store);

static int htu21d_init(struct htu21d_dev *htu21d)
{
    int ret = 0;

    ret = htu21d_reset(htu21d);
    if (ret) {
        return ret;
    }

    ret = htu21d_config(htu21d);
    if (ret) {
        return ret;
    }

    return 0;
}

static int htu21d_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    struct htu21d_dev *htu21d;
    int ret = 0;

    htu21d = devm_kzalloc(&client->dev, sizeof(*htu21d), GFP_KERNEL);
    if (IS_ERR(htu21d)) {
        dev_err(&client->dev, "kzalloc error: %ld\n", -PTR_ERR(htu21d));
        return PTR_ERR(htu21d);
    }

    i2c_set_clientdata(client, htu21d);
    htu21d->client = client;

    ret = htu21d_init(htu21d);
    if (ret) {
        dev_err(&client->dev, "htu21d init error: %d\n", -ret);
        return ret;
    }

    device_create_file(&client->dev, &dev_attr_temperature);
    device_create_file(&client->dev, &dev_attr_humidity);

    dev_info(&client->dev, "htu21d probe success\n");
    return 0;
}

static int htu21d_remove(struct i2c_client *client)
{
    i2c_set_clientdata(client, NULL);
    device_remove_file(&client->dev, &dev_attr_temperature);
    device_remove_file(&client->dev, &dev_attr_humidity);

    dev_info(&client->dev, "htu21d remove\n");
    return 0;
}

static struct i2c_device_id htu21d_id_table[] = {
    {"se,htu21d", 0},
    { },
};
MODULE_DEVICE_TABLE(i2c, htu21d_id_table);

static struct of_device_id htu21d_of_match[] = {
    {.compatible = "se,htu21d"},
    { },
};
MODULE_DEVICE_TABLE(of, htu21d_of_match);

static struct i2c_driver htu21d_driver = {
    .probe = htu21d_probe,
    .remove = htu21d_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "htu21d",
        .of_match_table = of_match_ptr(htu21d_of_match),
    },
    .id_table = htu21d_id_table,
};

module_i2c_driver(htu21d_driver);

MODULE_AUTHOR("duapple <duapple2@gmail.com>");
MODULE_LICENSE("GPL");