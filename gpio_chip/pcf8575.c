#include "linux/device.h"
#include "linux/gpio/driver.h"
#include "linux/interrupt.h"
#include "linux/irq.h"
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>

#define GPIO_NUM 16

struct pcf8575_dev {
    struct i2c_client *client;
    struct gpio_chip chip;

    u16 value;
    u16 in_mask;
    u16 r_val;

    struct irq_domain *irq_domain;
};

static int pcf8575_i2c_read(struct i2c_client *client, u16 *value)
{
    u8 data[2] = {0};
    struct i2c_msg msg[] = {
        {
            .flags = I2C_M_RD | I2C_M_STOP,
            .addr = client->addr,
            .buf = data,
            .len = ARRAY_SIZE(data),
        },
    };

    if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
        dev_err(&client->dev, "i2c read error, chip=0x%02x\n", client->addr);
        return -EREMOTEIO;
    }

    *value = data[0] | (data[1] << 8);
    return 0;
}

static int pcf8575_i2c_write(struct i2c_client *client, u16 value)
{
    u8 data[2] = {(value & 0xFF), ((value >> 8) & 0xFF)};
    struct i2c_msg msg[] = {
        {
            .flags = I2C_M_STOP,
            .addr = client->addr,
            .buf = data,
            .len = ARRAY_SIZE(data),
        },
    };

    if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
        dev_err(&client->dev, "i2c write error, chip=0x%02x\n", client->addr);
        return -EREMOTEIO;
    }

    return 0;
}

static int pcf8575_get_value(struct gpio_chip *chip, unsigned offset)
{
    u16 value = 0;
    struct pcf8575_dev *pcf8575 = container_of(chip, struct pcf8575_dev, chip);

    if (offset >= GPIO_NUM) {
        dev_err(&pcf8575->client->dev, "offset over %d\n", GPIO_NUM);
        return 0;
    }
    
    pcf8575_i2c_read(pcf8575->client, &value);
    pcf8575->r_val = value & pcf8575->in_mask;

    return (value & (1 << offset));
}

static void pcf8575_set_value(struct gpio_chip *chip, unsigned offset, int value)
{
    struct pcf8575_dev *pcf8575 = container_of(chip, struct pcf8575_dev, chip);

    if (offset >= GPIO_NUM) {
        dev_err(&pcf8575->client->dev, "offset over %d\n", GPIO_NUM);
        return ;
    }

    if (value) {
        pcf8575->value |= (1 << offset);
    } else {
        pcf8575->value &= (~(1 << offset));
    }

    pcf8575_i2c_write(pcf8575->client, pcf8575->value);
}

static int pcf8575_direction_input(struct gpio_chip *chip, unsigned offset)
{
    struct pcf8575_dev *pcf8575 = container_of(chip, struct pcf8575_dev, chip);
    pcf8575->value |= (1 << offset);
    pcf8575->in_mask |= (1 << offset);

    pcf8575_i2c_write(pcf8575->client, pcf8575->value);

    return 0;
}

static int pcf8575_direction_output(struct gpio_chip *chip, unsigned offset,
                                    int value)
{
    struct pcf8575_dev *pcf8575 = container_of(chip, struct pcf8575_dev, chip);
    pcf8575->in_mask &= (~(1 << offset));
    pcf8575_set_value(chip, offset, value);

    return 0;
}

static int pcf8575_irq_domain_map(struct irq_domain *domain, unsigned int virq,
                irq_hw_number_t hw)
{
    irq_set_chip_and_handler(virq, &dummy_irq_chip, handle_edge_irq);
    return 0;
}

static struct irq_domain_ops pcf8575_irq_domain_ops = {
    .map = pcf8575_irq_domain_map,
    .xlate = irq_domain_xlate_twocell,
};

static irqreturn_t pcf8575_irq(int irq, void *data)
{
    struct pcf8575_dev *pcf8575 = data;
    unsigned int child_irq, i;
    u16 value;

    pcf8575_i2c_read(pcf8575->client, &value);
    // pr_info("pcf8575 irq handle, value:0x%02x, r_val:0x%02x\n", value, pcf8575->r_val);

    for (i = 0; i < pcf8575->chip.ngpio; i++) {
        if (((pcf8575->in_mask >> i) & 0x01) == 0x01) {
            if (((value >> i) & 0x01) != ((pcf8575->r_val >> i) & 0x01)) {
                // pr_info("pcf8575 irq pin: %d", i);
                child_irq = irq_find_mapping(pcf8575->irq_domain, i);
                handle_nested_irq(child_irq);
            }
        }
    }

    /* If only we read from ports, we must cache last input value after using pcf8575_i2c_read */
    pcf8575->r_val = value & pcf8575->in_mask;
    return IRQ_HANDLED;
}

static int pcf8575_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
    struct pcf8575_dev *pcf8575 = container_of(chip, struct pcf8575_dev, chip);

	return irq_find_mapping(pcf8575->irq_domain, offset);
}

static int pcf8575_irq_setup(struct pcf8575_dev *pcf8575)
{
    struct device *dev = &pcf8575->client->dev;
    int ret = 0;
    int i = 0;
    u32 irq;

    pcf8575->in_mask = 0;
    pcf8575->r_val = 0;

    // pcf8575->client->irq = irq_of_parse_and_map(pcf8575->client->dev.of_node, 0);

    pcf8575->irq_domain = irq_domain_add_linear(dev->of_node,
        pcf8575->chip.ngpio, &pcf8575_irq_domain_ops, NULL);
    if (IS_ERR(pcf8575->irq_domain)) {
        return PTR_ERR(pcf8575->irq_domain);
    }

    ret = devm_request_threaded_irq(dev, pcf8575->client->irq, NULL, pcf8575_irq,
        IRQF_ONESHOT | IRQF_TRIGGER_FALLING, dev_name(dev), pcf8575);
    if (ret) {
        return ret;
    }

    pcf8575->chip.to_irq = pcf8575_gpio_to_irq;

    for (i = 0; i < pcf8575->chip.ngpio; i++) {
        irq = irq_create_mapping(pcf8575->irq_domain, i);
    }
    return 0;
}

static int pcf8575_irq_remove(struct pcf8575_dev *pcf8575)
{
    int i;
    int virq;

    for (i = 0; i < pcf8575->chip.ngpio; i++) {
        virq = pcf8575_gpio_to_irq(&pcf8575->chip, i);
        irq_dispose_mapping(virq);
    }

    irq_domain_remove(pcf8575->irq_domain);
    return 0;
}

static int pcf8575_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    struct pcf8575_dev *pcf8575;
    int ret;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "i2c_check_functionality not support I2C_FUNC_I2C\n");
        return -EIO;
    }

    pcf8575 = devm_kzalloc(&client->dev, sizeof(struct pcf8575_dev), GFP_KERNEL);
    if (IS_ERR(pcf8575)) {
        dev_err(&client->dev, "kzalloc error: 0x%lx\n", PTR_ERR(pcf8575));
        return PTR_ERR(pcf8575);
    }

    pcf8575->chip.label = client->name;
    pcf8575->chip.base = -1;
    pcf8575->chip.parent = &client->dev;
    pcf8575->chip.owner = THIS_MODULE,
    pcf8575->chip.ngpio = GPIO_NUM;
    pcf8575->chip.can_sleep = 1;
    pcf8575->chip.get = pcf8575_get_value,
    pcf8575->chip.set = pcf8575_set_value,
    pcf8575->chip.direction_input = pcf8575_direction_input,
    pcf8575->chip.direction_output = pcf8575_direction_output,

    /* default value is 0xFFFF after power on */
    pcf8575->value = 0xFFFF;

    i2c_set_clientdata(client, pcf8575);
    pcf8575->client = client;

    ret = devm_gpiochip_add_data(&client->dev, &pcf8575->chip, NULL);
    if (ret) {
        return ret;
    }

    ret = pcf8575_irq_setup(pcf8575);
    if (ret) {
        return ret;
    }

    dev_info(&client->dev, "pcf8575 probe success\n");
    return 0;
}

static int pcf8575_remove(struct i2c_client *client)
{
    struct pcf8575_dev *pcf8575 = i2c_get_clientdata(client);
    pcf8575_irq_remove(pcf8575);
    return 0;
}

static struct i2c_device_id pcf8575_id_table[] = {
    {"pcf8575", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, pcf8575_id_table);

static const struct of_device_id pcf8575_of_match[] = {
    {.compatible = "nxp,pcf8575"},
    { },
};
MODULE_DEVICE_TABLE(of, pcf8575_of_match);

static struct i2c_driver pcf8575_driver = {
    .probe = pcf8575_probe,
    .remove = pcf8575_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "pcf8575_drv",
        .of_match_table = of_match_ptr(pcf8575_of_match),
    },
    .id_table = pcf8575_id_table,
};

module_i2c_driver(pcf8575_driver);

MODULE_AUTHOR("duapple <duapple2@gmail.com>");
MODULE_LICENSE("GPL");