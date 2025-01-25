#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>

struct bmp280_dev {
    struct spi_cleint *client;
}