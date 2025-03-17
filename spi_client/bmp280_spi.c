#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

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
    struct spi_device *spi;
    struct bmp280_compensation compensation;

    s32 adc_T;
    s32 adc_P;

    s32 temperature;
    u32 pressure;
};

typedef struct {
    u8 reg;
    u8 val;
} spi_w_ctrl;

typedef u32 BMP280_U32_t;
typedef s32 BMP280_S32_t;
typedef s64 BMP280_S64_t;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123”
// equals 51.23 DegC. t_fine carries fine temperature as global value
static BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T,
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
static BMP280_U32_t bmp280_compensate_P_int32(BMP280_S32_t adc_P,
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

static int bmp280_spi_read(struct spi_device *spi, u8 reg, u8 *data, size_t len)
{
#if 0
    return spi_write_then_read(spi, &reg, 1, data, len);
#else
    struct spi_transfer xfer = {0};
    u8 *tx_buf = kzalloc(sizeof(u8), len);
    u8 *rx_buf = kzalloc(sizeof(u8), len);
    struct spi_message msg = {0};
    int ret = 0;

    tx_buf[0] = reg;
    xfer.tx_buf = tx_buf;
    xfer.rx_buf = rx_buf;
    xfer.len = len + 1;
    // xfer.cs_change = 1;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);

    ret = spi_sync(spi, &msg);

    memcpy(data, rx_buf + 1, len);

    kfree(tx_buf);
    kfree(rx_buf);
    return ret;
#endif
}

static int bmp280_spi_write(struct spi_device *spi, spi_w_ctrl *wc, size_t len)
{
    struct spi_transfer xfer = {0};
    u8 *data = kzalloc(sizeof(u8), 2 * len);
    int i = 0;
    struct spi_message msg = {0};
    int ret = 0;

    for (i = 0; i < len; i++) {
        data[2 * i] = wc[i].reg & (~(1 << 7));
        data[2 * i + 1] = wc[i].val;
    }

    xfer.tx_buf = data;
    xfer.len = 2 * len;
    // xfer.cs_change = 1;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);

    ret = spi_sync(spi, &msg);

    kfree(data);
    return ret;
}

static int bmp280_compensation_load(struct bmp280_dev *bmp280)
{
    u8 data[24] = {0};
    struct spi_device *spi = bmp280->spi;
    u8 reg = 0x88;
    int ret = 0;
    struct bmp280_compensation *com = &bmp280->compensation;

    ret = bmp280_spi_read(spi, reg, data, sizeof(data));
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

    dev_info(&bmp280->spi->dev,
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
    int ret = 0;
    struct spi_device *spi = bmp280->spi;
    spi_w_ctrl w_ctrl = {
        .reg = 0xF4,
        .val = (0x07 << 5) | (0x07 << 2) | 0x01,
    };
    u8 reg = 0xF7;

    /* write config for sample, oversampling x1, force mode */
    bmp280_spi_write(spi, &w_ctrl, 1);

    /* measure time Max 6.4ms: T and P oversampling x1 */
    msleep(7);

    ret = bmp280_spi_read(spi, reg, data, sizeof(data));
    if (ret) {
        return ret;
    }

    bmp280->adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    bmp280->adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    dev_info(&bmp280->spi->dev, "adc_P: 0x%x, adc_T: 0x%x\n", bmp280->adc_P,
            bmp280->adc_T);
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

    dev_info(&bmp280->spi->dev, "temperature: %d, pressure: %u\n",
            bmp280->temperature, bmp280->pressure);
    return 0;
}

static ssize_t bmp280_temperature_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
    struct spi_device *spi = container_of(dev, struct spi_device, dev);
    struct bmp280_dev *bmp280 = spi_get_drvdata(spi);

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
    struct spi_device *spi = container_of(dev, struct spi_device, dev);
    struct bmp280_dev *bmp280 = spi_get_drvdata(spi);

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
    struct spi_device *spi = bmp280->spi;
    u8 id = 0;
    int ret = 0;
    u8 reg = 0xD0;
    spi_w_ctrl w_ctrl = {
        .reg = 0xE0,
        .val = 0xB6,
    };

    ret = bmp280_spi_read(spi, reg, &id, 1);
    if (ret < 0) {
        return -EREMOTEIO;
    }

    dev_info(&spi->dev, "Read BMP280 Device ID: 0x%02x\n", id);

    /* Reset chip */
    ret = bmp280_spi_write(spi, &w_ctrl, 1);
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

static int bmp280_probe(struct spi_device *spi)
{
    struct bmp280_dev *bmp280;
    int ret = 0;

    bmp280 = devm_kzalloc(&spi->dev, sizeof(*bmp280), GFP_KERNEL);
    if (IS_ERR(bmp280)) {
        dev_err(&spi->dev, "kzalloc fail: 0x%lx\n", PTR_ERR(bmp280));
        return PTR_ERR(bmp280);
    }

    spi_set_drvdata(spi, bmp280);
    bmp280->spi = spi;

    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 5000000;
    ret = spi_setup(spi);
    if (ret) {
        return -EIO;
    }

    if (bmp280_init(bmp280)) {
        dev_err(&spi->dev, "bmp280 init error\n");
        return -EIO;
    }

    device_create_file(&spi->dev, &dev_attr_temperature);
    device_create_file(&spi->dev, &dev_attr_pressure);

    dev_info(&spi->dev, "bmp280 probe success\n");
    return 0;
}

static int bmp280_remove(struct spi_device *spi)
{
    spi_set_drvdata(spi, NULL);
    device_remove_file(&spi->dev, &dev_attr_temperature);
    device_remove_file(&spi->dev, &dev_attr_pressure);

    dev_info(&spi->dev, "bmp280 remove");
    return 0;
}

static struct of_device_id bmp280_of_match[] = {
    {.compatible = "bosch,bmp280-spi"},
    { },
};
MODULE_DEVICE_TABLE(of, bmp280_of_match);

static struct spi_driver bmp280_driver = {
    .probe = bmp280_probe,
    .remove = bmp280_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "bmp280",
        .of_match_table = of_match_ptr(bmp280_of_match),
    },
};

module_spi_driver(bmp280_driver);

MODULE_AUTHOR("duapple <duapple2@gmail.com>");
MODULE_LICENSE("GPL");