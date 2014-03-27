#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/types.h>

#include "log.h"

MODULE_AUTHOR("Kevin Liu (airk908@gmail.com)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HMC5883L i2c driver for module");

#define HMC5883L_REG_CONFIG_A   0
    #define SAMPLE_AVERAGE_OFFSET   5
enum {
    SAMPLE_AVERAGE_1,
    SAMPLE_AVERAGE_2,
    SAMPLE_AVERAGE_4,
    SAMPLE_AVERAGE_8,
    SAMPLE_AVERAGE_MAX = SAMPLE_AVERAGE_8
};
    #define OUTPUT_RATE_OFFSET      2
enum {
    /* bit | Hz */
    OUTPUT_RATE_0_75,
    OUTPUT_RATE_1_5,
    OUTPUT_RATE_3,
    OUTPUT_RATE_7_5,
    OUTPUT_RATE_15,
    OUTPUT_RATE_30,
    OUTPUT_RATE_75,
    OUTPUT_RATE_MAX = OUTPUT_RATE_75
};
    #define MESURA_NORMAL_OFFSET    0
enum {
    /* bit | mesura */
    MESURA_NORMAL,
    MESURA_POSITIVE,
    MESURA_NEGITIVE,
    MESURA_MAX = MESURA_NEGITIVE
};

#define HMC5883L_REG_CONFIG_B   1
    #define GAIN_OFFSET             5
enum {
    /* bit | recommended | gain */
    GAIN_0_88_1370,
    GAIN_1_3_1090,
    GAIN_1_9_820,
    GAIN_2_5_660,
    GAIN_4_0_440,
    GAIN_4_7_390,
    GAIN_5_6_330,
    GAIN_8_1_230,
    GAIN_MAX = GAIN_8_1_230
};

#define HMC5883L_REG_MODE       2
    #define MODE_OFFSET             0
enum {
    /* bit | mode */
    MODE_CONTINOUS,
    MODE_SINGLE,
    MODE_IDLE,
    MODE_MAX = MODE_IDLE
};

#define HMC5883L_REG_XDATA_MSB  3
#define HMC5883L_REG_XDATA_LSB  4
#define HMC5883L_REG_YDATA_MSB  5
#define HMC5883L_REG_YDATA_LSB  6
#define HMC5883L_REG_ZDATA_MSB  7
#define HMC5883L_REG_ZDATA_LSB  8

#define HMC5883L_REG_STATUS     9
#define HMC5883L_VERSION_ID_A   10
#define HMC5883L_VERSION_ID_B   11
#define HMC5883L_VERSION_ID_C   12

enum {
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    AXIS_MAX = AXIS_Z
};

struct hmc5883l_sensor {
    struct mutex lock;
    struct i2c_client *client;
    struct delayed_work work;
    struct input_dev *input;

    unsigned long delay_ms;
    int enable;

    int sample_average;
    int output_rate;
    int mesura;
    int gain;
    int mode;
    int axis[AXIS_MAX + 1];

    bool is_self_test;
    bool is_validation;
};

static void hmc5883l_write(struct hmc5883l_sensor *hmc5883l,
        u8 reg, u8 val)
{
    i2c_smbus_write_byte_data(hmc5883l->client, reg, val);
}

static s32 hmc5883l_read(struct hmc5883l_sensor *hmc5883l, u8 reg)
{
    return i2c_smbus_read_byte_data(hmc5883l->client, reg);
}

static void hmc5883l_enable(struct hmc5883l_sensor *hmc5883l)
{
    if (hmc5883l->mode <= MODE_MAX && hmc5883l->mode >= 0) {
        mutex_lock(&hmc5883l->lock);
        hmc5883l_write(hmc5883l, HMC5883L_REG_MODE,
                hmc5883l->mode << MODE_OFFSET);
        hmc5883l->enable = 1;
        mutex_unlock(&hmc5883l->lock);

        schedule_delayed_work(&hmc5883l->work,
                msecs_to_jiffies(hmc5883l->delay_ms));
    } else
        E("Invalid mode %d, skip.\n", hmc5883l->mode);
}

static void hmc5883l_disable(struct hmc5883l_sensor *hmc5883l)
{
    mutex_lock(&hmc5883l->lock);
    hmc5883l_write(hmc5883l, HMC5883L_REG_MODE,
            MODE_IDLE << MODE_OFFSET);
    hmc5883l->enable = 0;
    mutex_unlock(&hmc5883l->lock);
}

#if 0
static void hmc5883l_set_gain(struct hmc5883l_sensor *hmc5883l,
        int gain)
{
    gain &= GAIN_MAX;
    if (gain <= GAIN_0_88_1370 && gain >= GAIN_8_1_230) {
        mutex_lock(&hmc5883l->lock);
        hmc5883l->gain = gain;
        hmc5883l_write(hmc5883l, HMC5883L_REG_CONFIG_B,
                hmc5883l->gain << GAIN_OFFSET);
        mutex_unlock(&hmc5883l->lock);
    } else
        E("Invalid gain %d, skip\n", gain);
}

static int hmc5883l_get_gain(struct hmc5883l_sensor *hmc5883l)
{
    return (int) (hmc5883l_read(hmc5883l,
                HMC5883L_REG_CONFIG_B) >> GAIN_OFFSET & 0xfff);
}
#endif

static int hmc5883l_get_xyz_data(struct hmc5883l_sensor *hmc5883l,
        s16 data[])
{
    int ret;
    int i;
    s16 values[AXIS_MAX + 1];

    ret = i2c_smbus_read_i2c_block_data(hmc5883l->client,
            HMC5883L_REG_XDATA_MSB, 6, (u8 *)values);
    if (ret < 0) {
        E("error read data %d\n", ret);
        return ret;
    }

    for (i = 0; i < AXIS_MAX + 1; i++) {
        values[i] = be16_to_cpu(values[i]);
    }

    hmc5883l->axis[AXIS_X] = values[AXIS_X];
    hmc5883l->axis[AXIS_Y] = values[AXIS_Y];
    hmc5883l->axis[AXIS_Z] = values[AXIS_Z];

    D("x:%d, y:%d, z:%d\n", values[AXIS_X],
            values[AXIS_Y], values[AXIS_Z]);
    return 0;
}

static void hmc5883l_workqueue(struct work_struct *work)
{
    int ret;
    struct hmc5883l_sensor *hmc5883l = container_of((struct delayed_work *) work,
            struct hmc5883l_sensor, work);

    s16 axis[AXIS_MAX +1];

    mutex_lock(&hmc5883l->lock);

    if (!hmc5883l->enable)
        goto out;

    ret = hmc5883l_get_xyz_data(hmc5883l, axis);
    if (ret < 0) {
        E("error read data from hmc5883l_get_xyz_data %d\n", ret);
    } else {
        input_report_abs(hmc5883l->input, ABS_X, axis[AXIS_X]);
        input_report_abs(hmc5883l->input, ABS_Y, axis[AXIS_Y]);
        input_report_abs(hmc5883l->input, ABS_Z, axis[AXIS_Z]);
        input_sync(hmc5883l->input);
    }

    schedule_delayed_work(&hmc5883l->work,
            msecs_to_jiffies(hmc5883l->delay_ms));
out:
    mutex_unlock(&hmc5883l->lock);
}

#define MIN_DELAY_MS (1000 / 75 + 5)
static ssize_t delay_set(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct hmc5883l_sensor *hmc5883l = dev_get_drvdata(dev);
    unsigned long delay_ms = 0;

    if (strict_strtoul(buf, 10, &delay_ms) || delay_ms == 0) {
        E("Invalid delay ms %lu, skip\n", delay_ms);
        return -EINVAL;
    }

    if (delay_ms < MIN_DELAY_MS)
        delay_ms = MIN_DELAY_MS;

    mutex_lock(&hmc5883l->lock);
    hmc5883l->delay_ms = delay_ms;
    mutex_unlock(&hmc5883l->lock);

    return size;
}

static ssize_t delay_get(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct hmc5883l_sensor *hmc5883l = dev_get_drvdata(dev);
    int delay_ms;

    mutex_lock(&hmc5883l->lock);
    delay_ms = hmc5883l->delay_ms;
    mutex_unlock(&hmc5883l->lock);

    return sprintf(buf, "%d\n", delay_ms);
}
static DEVICE_ATTR(delay, 0664, delay_get, delay_set);

static ssize_t enable_get(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct hmc5883l_sensor *hmc5883l = dev_get_drvdata(dev);
    int val;

    mutex_lock(&hmc5883l->lock);
    val = hmc5883l->enable;
    mutex_unlock(&hmc5883l->lock);

    return sprintf(buf, "%d\n", val);
}

static ssize_t enable_set(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct hmc5883l_sensor *hmc5883l = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 10, &val))
        return -EINVAL;

    D("enable_set: %lu\n", val);

    if (val)
        hmc5883l_enable(hmc5883l);
    else
        hmc5883l_disable(hmc5883l);

    hmc5883l->enable = !!val;

    return size;
}
static DEVICE_ATTR(enable, 0664, enable_get, enable_set);

static int hmc5883l_input_init(struct hmc5883l_sensor *hmc5883l)
{
    struct input_dev *dev;
    int ret;

    dev = input_allocate_device();
    if (!dev)
        return -ENOMEM;

    dev->name = "hmc5883l-compass";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_REL, dev->evbit);
    set_bit(REL_X, dev->relbit);
    set_bit(REL_Y, dev->relbit);
    set_bit(REL_Z, dev->relbit);
    input_set_drvdata(dev, hmc5883l);

    ret = input_register_device(dev);
    if (ret < 0) {
        input_free_device(dev);
        return ret;
    }

    hmc5883l->input = dev;
    return 0;
}

static struct attribute *hmc5883l_attributes[] = {
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    NULL
};

static struct attribute_group hmc5883l_attr_group = {
    .name = "hmc5883l-i2c",
    .attrs = hmc5883l_attributes
};

static int hmc5883l_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret = 0;
    struct hmc5883l_sensor *hmc5883l;
    u8 id1, id2, id3;
    D("Probe match, id name %s\n", id->name);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        E("i2c_check_functionality error\n");
        ret = -EINVAL;
        goto out;
    }

    hmc5883l = kzalloc(sizeof(*hmc5883l), GFP_KERNEL);
    if (!hmc5883l) {
        ret = -ENOMEM;
        goto out;
    }

    hmc5883l->client = client;
    i2c_set_clientdata(client, hmc5883l);
    hmc5883l->delay_ms = 100;
    mutex_init(&hmc5883l->lock);
    INIT_DELAYED_WORK(&hmc5883l->work, hmc5883l_workqueue);

    id1 = hmc5883l_read(hmc5883l, HMC5883L_VERSION_ID_A);
    id2 = hmc5883l_read(hmc5883l, HMC5883L_VERSION_ID_B);
    id3 = hmc5883l_read(hmc5883l, HMC5883L_VERSION_ID_C);
    if (!(id1 == 'H' && id2 == '4' && id3 == '3')) {
        E("chip version not correct(%c%c%c)\n", id1, id2, id3);
        goto out_sensor;
    }

    hmc5883l_disable(hmc5883l);

    ret = hmc5883l_input_init(hmc5883l);
    if (ret < 0) {
        E("input sub class init failed %d\n", ret);
        goto out_sensor;
    }

    ret = sysfs_create_group(&client->dev.kobj, &hmc5883l_attr_group);
    if (ret < 0) {
        E("sysfs create failed %d\n", ret);
        goto out_free_input;
    }

    //hmc5883l_init_regs(hmc5883l);
    //hmc5883l_enable(hmc5883l);

    return 0;

out_free_input:
    input_unregister_device(hmc5883l->input);
out_sensor:
    kfree(hmc5883l);
out:
    return ret;
}

static int hmc5883l_remove(struct i2c_client *client)
{
    struct hmc5883l_sensor *hmc5883l = i2c_get_clientdata(client);

    hmc5883l_disable(hmc5883l);
    sysfs_remove_group(&client->dev.kobj, &hmc5883l_attr_group);
    input_unregister_device(hmc5883l->input);
    kfree(hmc5883l);

    return 0;
}

static struct i2c_device_id hmc5883l_id[] = {
    {"hmc5883l-i2c", 0},
    { }
};

static struct i2c_driver hmc5883l_driver = {
    .driver = {
        .name = "hmc5883l-i2c",
        .owner = THIS_MODULE,
    },
    .class = I2C_CLASS_HWMON,
    .id_table = hmc5883l_id,
    .probe = hmc5883l_probe,
    .remove = hmc5883l_remove,
};

module_i2c_driver(hmc5883l_driver);
