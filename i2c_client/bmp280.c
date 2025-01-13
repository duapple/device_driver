#include "asm-generic/errno-base.h"
#include "asm-generic/errno.h"
#include "linux/device.h"
#include "linux/i2c.h"
#include "linux/kernel.h"
#include "linux/slab.h"
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>

struct bmp280_compensation {
    unsigned short dig_T1;
    signed short dig_T2;
    signed short dig_T3;
    unsigned short dig_P1;
    signed short dig_P2;
    signed short dig_P3;
    signed short dig_P4;
    signed short dig_P5;
    signed short dig_P6;
    signed short dig_P7;
    signed short dig_P8;
    signed short dig_P9;

    s32 t_fine;
};

struct bmp280_dev {
    struct i2c_client *client;
    struct bmp280_compensation compensation;

    s32 adc_T;
    s32 adc_P;

    s32 temperature;
    u32 pressure;
};

/* auto increase */
struct bmp280_i2c_reg_ctrl_r {
    u8 reg;
    u8 *val;
    u16 len;
};

/* can't auto increase */
struct bmp280_i2c_reg_ctrl_w {
    u8 reg;
    u8 val;
};

static int bmp280_i2c_write_reg(struct i2c_client *client, u16 chip, u8 reg, u8 value)
{
    u8 data[] = {reg, value};
    struct i2c_msg msg[] = {
        {
            .flags = I2C_M_STOP,
            .addr = chip,
            .buf = data,
            .len = ARRAY_SIZE(data),
        },
    };

    if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) < 0) {
        dev_err(&client->dev, "i2c write reg error, chip=0x%02x, reg=0x%02x, val=0x%02x\n",
                chip, reg, value);
    }

    return 0;
}

static int bmp280_i2c_read_reg(struct i2c_client *client, u16 chip, u8 reg, u8 *value)
{
    struct i2c_msg msg[] = {
        {
            .flags = 0,
            .addr = chip,
            .buf = &reg,
            .len = 1,
        },
        {
            .flags = I2C_M_RD | I2C_M_STOP,
            .addr = chip,
            .buf = value,
            .len = 1,
        },
    };

    if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) < 0) {
        dev_err(&client->dev, "i2c read reg error, chip=0x%02x, reg=0x%02x",
                chip, reg);
        return -EIO;
    }

    return 0;
}

static int bmp280_i2c_multi_bytes_write(struct i2c_client *client, u16 chip,
                                        struct bmp280_i2c_reg_ctrl_w *reg_ctrl, u16 len)
{
    struct i2c_msg msg;
    int i = 0;
    u8 *data;

    data = kzalloc(sizeof(u8) * len * 2, GFP_KERNEL);
    if (IS_ERR(data)) {
        return PTR_ERR(data);
    }

    for (i = 0; i < len; i++) {
        data[i * 2] = reg_ctrl[i].reg;
        data[1 * 2 + 1] = reg_ctrl[i].val;
    }

    msg.flags = I2C_M_STOP;
    msg.addr = chip;
    msg.buf = data;
    msg.len = len * 2;

    if (i2c_transfer(client->adapter, &msg, 1) < 0) {
        dev_err(&client->dev, "i2c multi bytes write error, chip=0x%02x\n", chip);
        kfree(data);
        return -EIO;
    }

    kfree(data);
    return 0;
}

static int bmp280_i2c_multi_bytes_read(struct i2c_client *client, u16 chip,
                                        struct bmp280_i2c_reg_ctrl_r *reg_ctrl)
{
    struct i2c_msg msg[] = {
        {
            .flags = 0,
            .addr = chip,
            .buf = &reg_ctrl->reg,
            .len = 1,
        },
        {
            .flags = I2C_M_RD | I2C_M_STOP,
            .addr = chip,
            .buf = reg_ctrl->val,
            .len = reg_ctrl->len,
        },
    };

    if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) < 0) {
        dev_err(&client->dev, "i2c multi bytes read error, chip=0x%02x\n", chip);
        return -EIO;
    }

    return 0;
}

typedef u32 BMP280_U32_t;
typedef s32 BMP280_S32_t;
typedef s64 BMP280_S64_t;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123”
// equals 51.23 DegC. t_fine carries fine temperature as global value
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T,
                                       struct bmp280_compensation *com)
{
    BMP280_S32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((BMP280_S32_t)com->dig_T1 << 1))) *
            ((BMP280_S32_t)com->dig_T2)) >>
           11;
    var2 = (((((adc_T >> 4) - ((BMP280_S32_t)com->dig_T1)) *
              ((adc_T >> 4) - ((BMP280_S32_t)com->dig_T1))) >>
             12) *
            ((BMP280_S32_t)com->dig_T3)) >>
           14;
    com->t_fine = var1 + var2;
    T = (com->t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386”
// equals 96386 Pa = 963.86 hPa
BMP280_U32_t bmp280_compensate_P_int32(BMP280_S32_t adc_P,
                                        struct bmp280_compensation *com) 
{
    BMP280_S32_t var1, var2;
    BMP280_U32_t p;
    var1 = (((BMP280_S32_t)com->t_fine) >> 1) - (BMP280_S32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((BMP280_S32_t)com->dig_P6);
    var2 = var2 + ((var1 * ((BMP280_S32_t)com->dig_P5)) << 1);
    var2 = (var2 >> 2) + (((BMP280_S32_t)com->dig_P4) << 16);
    var1 = (((com->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((BMP280_S32_t)com->dig_P2) * var1) >> 1)) >>
           18;
    var1 = ((((32768 + var1)) * ((BMP280_S32_t)com->dig_P1)) >> 15);
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = (((BMP280_U32_t)(((BMP280_S32_t)1048576) - adc_P) - (var2 >> 12))) *
        3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((BMP280_U32_t)var1);
    } else {
        p = (p / (BMP280_U32_t)var1) * 2;
    }
    var1 = (((BMP280_S32_t)com->dig_P9) *
            ((BMP280_S32_t)(((p >> 3) * (p >> 3)) >> 13))) >>
           12;
    var2 = (((BMP280_S32_t)(p >> 2)) * ((BMP280_S32_t)com->dig_P8)) >> 13;
    p = (BMP280_U32_t)((BMP280_S32_t)p + ((var1 + var2 + com->dig_P7) >> 4));
    return p;
}

static int bmp280_compensation_load(struct bmp280_dev *bmp280)
{
    u8 data[24] = {0};
    struct i2c_client *client = bmp280->client;
    struct bmp280_i2c_reg_ctrl_r reg_ctrl = {
        .reg = 0x88,
        .val = data,
        .len = ARRAY_SIZE(data),
    };
    int ret = 0;
    struct bmp280_compensation *com = &bmp280->compensation;

    ret = bmp280_i2c_multi_bytes_read(client, client->addr, &reg_ctrl);
    if (ret) {
        return ret;
    }

    com->dig_T1 = data[0] | (data[1] << 8);
    com->dig_T2 = data[2] | (data[3] << 8);
    com->dig_T3 = data[4] | (data[5] << 8);
    com->dig_P1 = data[6] | (data[7] << 8);
    com->dig_P2 = data[8] | (data[9] << 8);
    com->dig_P3 = data[10] | (data[11] << 8);
    com->dig_P4 = data[12] | (data[13] << 8);
    com->dig_P5 = data[14] | (data[15] << 8);
    com->dig_P6 = data[16] | (data[17] << 8);
    com->dig_P7 = data[18] | (data[19] << 8);
    com->dig_P8 = data[20] | (data[21] << 8);
    com->dig_P9 = data[22] | (data[23] << 8);

    dev_info(&bmp280->client->dev,
            "dig_T1=0x%x\n"
            "dig_T2=0x%x\n"
            "dig_T3=0x%x\n"
            "dig_P1=0x%x\n"
            "dig_P2=0x%x\n"
            "dig_P3=0x%x\n"
            "dig_P4=0x%x\n"
            "dig_P5=0x%x\n"
            "dig_P6=0x%x\n"
            "dig_P7=0x%x\n"
            "dig_P8=0x%x\n"
            "dig_P9=0x%x\n",
            com->dig_T1, com->dig_T2, com->dig_T3,
            com->dig_P1, com->dig_P2, com->dig_P3,
            com->dig_P4, com->dig_P5, com->dig_P6,
            com->dig_P7, com->dig_P8, com->dig_P9);
    return 0;
}

static int bmp280_read_adc(struct bmp280_dev *bmp280)
{
    u8 data[6] = {0};
    struct bmp280_i2c_reg_ctrl_r reg_ctrl = {
        .reg = 0xF7,
        .val = data,
        .len = ARRAY_SIZE(data),
    };
    int ret = 0;
    struct i2c_client *client = bmp280->client;

    /* write config for sample, oversampling x1, force mode */
    bmp280_i2c_write_reg(client, client->addr, 0xF4, (0x07 << 5) | (0x07 << 2) | 0x01);

    /* measure time Max 6.4ms: T and P oversampling x1 */
    msleep(7);

    ret = bmp280_i2c_multi_bytes_read(client, client->addr, &reg_ctrl);
    if (ret) {
        return ret;
    }

    bmp280->adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    bmp280->adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    dev_info(&client->dev, "adc_P: 0x%x, adc_T: 0x%x\n", bmp280->adc_P, bmp280->adc_T);
    return 0;
}

static int bmp280_read_temp_hum_press(struct bmp280_dev *bmp280)
{
    int ret = 0;

    ret = bmp280_read_adc(bmp280);
    if (ret) {
        return ret;
    }

    bmp280->temperature = bmp280_compensate_T_int32(bmp280->adc_T,
                                                    &bmp280->compensation);
    bmp280->pressure = bmp280_compensate_P_int32(bmp280->adc_P,
                                                &bmp280->compensation);

    dev_info(&bmp280->client->dev, "temperature: %d, pressure: %u\n",
            bmp280->temperature, bmp280->pressure);
    return 0;
}

static ssize_t bmp280_temperature_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct bmp280_dev *bmp280 = i2c_get_clientdata(client);

    if (IS_ERR(bmp280)) {
        dev_err(dev, "i2c get cleintdata error: 0x%lx\n", PTR_ERR(bmp280));
    }

    bmp280_read_temp_hum_press(bmp280);
    return sprintf(buf, "%d\n", bmp280->temperature);
}

static ssize_t bmp280_temperature_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
    return 0;
}
static DEVICE_ATTR(temperature, 0644, bmp280_temperature_show,
                    bmp280_temperature_store);

static ssize_t bmp280_pressure_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct bmp280_dev *bmp280 = i2c_get_clientdata(client);

    if (IS_ERR(bmp280)) {
        dev_err(dev, "i2c get cleintdata error: 0x%lx\n", PTR_ERR(bmp280));
    }

    bmp280_read_temp_hum_press(bmp280);
    return sprintf(buf, "%u\n", bmp280->pressure);
}

static ssize_t bmp280_pressure_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
    return 0;
}
static DEVICE_ATTR(pressure, 0644, bmp280_pressure_show, bmp280_pressure_store);

static int bmp280_init(struct bmp280_dev *bmp280)
{
    struct i2c_client *client = bmp280->client;
    u8 id = 0;
    int ret = 0;

    ret = bmp280_i2c_read_reg(client, client->addr, 0xD0, &id);
    if (ret < 0) {
        return -EREMOTEIO;
    }

    dev_info(&client->dev, "Read BMP280 Device ID: 0x%02x\n", id);

    /* Reset chip */
    ret = bmp280_i2c_write_reg(client, client->addr, 0xE0, 0xB6);
    if (ret) {
        return ret;
    }

    /* startup time: 2ms */
    msleep(2);

    ret = bmp280_compensation_load(bmp280);
    if (ret) {
        return ret;
    }

    return 0;
}

static int bmp280_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct bmp280_dev *bmp280;

    bmp280 = devm_kzalloc(&client->dev, sizeof(*bmp280), GFP_KERNEL);
    if (IS_ERR(bmp280)) {
        dev_err(&client->dev, "kzalloc fail: 0x%lx\n", PTR_ERR(bmp280));
        return PTR_ERR(bmp280);
    }

    i2c_set_clientdata(client, bmp280);
    bmp280->client = client;

    if (bmp280_init(bmp280)) {
        dev_err(&client->dev, "bmp280 init error\n");
        return -EIO;
    }

    device_create_file(&client->dev, &dev_attr_temperature);
    device_create_file(&client->dev, &dev_attr_pressure);

    (void)bmp280_i2c_multi_bytes_write;
    (void)bmp280_i2c_multi_bytes_read;

    dev_info(&client->dev, "bmp280 probe success\n");
    return 0;
}

static int bmp280_remove(struct i2c_client *client)
{
    i2c_set_clientdata(client, NULL);
    device_remove_file(&client->dev, &dev_attr_temperature);
    device_remove_file(&client->dev, &dev_attr_pressure);

    dev_info(&client->dev, "bmp280 remove");
    return 0;
}

static struct i2c_device_id bmp280_id_table[] = {
    {"bosch,bmp280", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, bmp280_id_table);

static const struct of_device_id bmp280_of_match[] = {
    {.compatible = "bosch,bmp280"},
    { },
};
MODULE_DEVICE_TABLE(of, bmp280_of_match);

static struct i2c_driver bmp280_driver = {
    .probe = bmp280_probe,
    .remove = bmp280_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "bmp280",
        .of_match_table = of_match_ptr(bmp280_of_match),
    },
    .id_table = bmp280_id_table,
};

module_i2c_driver(bmp280_driver);

MODULE_AUTHOR("duapple <duapple2@gmail.com>");
MODULE_LICENSE("GPL");
