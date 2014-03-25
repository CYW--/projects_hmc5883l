#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/types.h>

#include "log.h"

#define GPIO1_16 48
#define I2C_ADAPTER_ID 2

#define SENSOR_ID_STRING "H43"
#define SENSOR_NAME "hmc5883l"
#define SENSOR_SLAVE_ADDRESS 0x1E
#define DRDY_INT GPIO1_16

#define HMC5883L_CONFIG_REG_A    0x00
    #define TOBE_CLEAR      1<<7
    #define SAMPLE_AVER     0x3
        #define SAMPLE_AVER_OFFSET   5
    #define DATA_OUT_RATE   0x7
        #define DATA_OUT_RATE_OFFSET 2
    #define MESURE_SETTING   3<<0

static int sample_average_data[] = {
    1, 2, 4, 8
};

static float data_out_rate[] = {
    0.75, 1.5, 3, 7.5, 15, 30, 75, 0
};

enum {
    NORMAL_MESURA = 0,
    POSITIVE_MESURA,
    NEGATIVE_MESURA,
    MAX_MESURA
};

#define HMC5883L_CONFIG_REG_B 0x01
    #define GAIN_SETTING 0x7
        #define GAIN_SETTING_OFFSET 5

static float gain_settings[] = {
    {0.88, 1370},
    {1.3, 1090},
    {1.9, 820},
    {2.5, 660},
    {4.0, 440},
    {4.7, 390},
    {5.6, 330},
    {8.1, 230},
};

#define HMC5883L_MODE_REG    0x02
    #define MODE_SETTING 3<<0

enum {
    CONTINOUS_MODE = 0,
    SINGLE_MODE,
    IDLE_MODE,
    MAX_MODE
};

#define HMC5883L_DATA_OUT_REG    0x03
#define HMC5883L_STATUS_REG      0x09
    #define DATA_LOCK       0x1
        #define DATA_LOCK_OFFSET 1
    #define DATA_READY      1<0

#define HMC5883L_IDENTIFY_REG_A  0x0A
#define HMC5883L_IDENTIFY_REG_B  0x0B
#define HMC5883L_IDENTIFY_REG_C  0x0C

struct sensor_hmc5883l {
    struct mutex lock;
    struct i2c_client *client;
    u16 x_axis;
    u16 y_axis;
    u16 z_axis;
    u8 sample;
    u8 out_rate;
    u8 mesura_set;
    u8 mode;
    u8 gain;
};
static struct sensor_hmc5883l *hmc5883l;

static s32 hmc5883l_write_byte(struct i2c_client *client,
        u8 reg, u8 val)
{
    return i2c_smbus_write_byte_data(client, reg, val);
}

static s32 hmc5883l_read_byte(struct i2c_client *client,
        u8 reg)
{
    return i2c_smbus_read_byte_data(client, reg);
}

static int hmc5883l_set_mode(struct i2c_client *client,
        u8 mode)
{
    s32 result;
    mode = mode & MODE_SETTING;
    if (mode >= MAX_MODE) {
        E("Invalid mode\n");
        return -EINVAL;
    }

    result = hmc5883l_write_byte(client, HMC5883L_MODE_REG, mode);
    return (result > 0? -EBUSY : result);
}

static s32 hmc5883l_get_mode(struct i2c_client *client)
{
    return hmc5883l_read_byte(client, HMC5883L_MODE_REG) & HMC5883L_MODE_REG;
}

static int hmc5883l_set_gain(struct i2c_client *client,
        u8 gain)
{
    s32 result;
    gain = gain & GAIN_SETTING;
    if (gain > 7 || gain < 0) {
        E("Invalid gain\n");
        return -EINVAL;
    }

    result = hmc5883l_write_byte(client, HMC5883L_CONFIG_REG_B, gain << GAIN_SETTING_OFFSET);
    return (result > 0? -EBUSY : result);
}

static u8 hmc5883l_get_gain(struct i2c_client *client)
{
    return hmc5883l_read_byte(client, HMC5883L_CONFIG_REG_B) >> GAIN_SETTING_OFFSET & GAIN_SETTING;
}

static s32 hmc5883l_set_mesura(struct i2c_client *client, u8 mesura)
{
    mesura = mesura & MESURE_SETTING;
    return hmc5883l_write_byte(client, HMC5883L_CONFIG_REG_A, mesura);
}

static u8 hmc5883l_get_mesura(struct i2c_client *client)
{
    s32 result;
    result = hmc5883l_read_byte(client, HMC5883L_CONFIG_REG_A);
    return (result & MESURE_SETTING);
}

static s32 hmc5883l_set_data_out_rate(struct i2c_client *client,
        u8 rate)
{
    rate = rate & DATA_OUT_RATE;
    if (rate >= 7 || rate < 0) {
        E("Invalid data output rate\n");
        return -EINVAL;
    }
//TODO
    return 0;
}

static s32 hmc5883l_get_version(struct i2c_client *client)
{
    s32 value = hmc5883l_read_byte(client, HMC5883L_CONFIG_REG_A);
    return value;
}

static bool hmc5883l_is_data_ready(struct i2c_client *client)
{
    s32 result;
    result = hmc5883l_read_byte(client, HMC5883L_STATUS_REG);
    if (result & DATA_READY)
        return true;
    else
        return false;
}

static bool hmc5883l_is_reg_locked(struct i2c_client *client)
{
    s32 result;
    result = hmc5883l_read_byte(client, HMC5883L_STATUS_REG);
    if (result >> DATA_LOCK_OFFSET & DATA_LOCK)
        return true;
    else
        return false;
}

static irqreturn_t hmc5883l_interrupt(int irq, void *dev_id)
{
    struct i2c_client *client = dev_id;

    mutex_lock(&hmc5883l->lock);

    if (hmc5883l_is_data_ready(client) && hmc5883l_is_reg_locked(client)) {
        hmc5883l->x_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG);
        hmc5883l->y_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG);
        hmc5883l->z_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG);
    } else
        D("Data not ready\n");

    mutex_unlock(&hmc5883l->lock);

    return IRQ_HANDLED;
}

static int hmc5883l_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    s32 version_value;

    version_value = hmc5883l_get_version(client);
    D("Chip version is %c\n", version_value & 0xff);

    return 0;
}

static int hmc5883l_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id hmc5883l_id[] = {
    {"hmc5883l_test", 0},
    { }
};
MODULE_DEVICE_TABLE(i2c, hmc5883l_id);

static struct i2c_driver hmc5883l_driver = {
    .driver = {
        .name = SENSOR_NAME,
    },
    .probe = hmc5883l_probe,
    .remove = hmc5883l_remove,
    .id_table = hmc5883l_id,
};

static struct i2c_board_info hmc5883l_device = {
    I2C_BOARD_INFO("hmc5883l_test", SENSOR_SLAVE_ADDRESS),
};

static int __init hmc5883l_init(void)
{
    struct i2c_adapter *adap;
    struct i2c_client *client;
    i2c_register_driver(THIS_MODULE, &hmc5883l_driver);

    D("i2c_register_drivers\n");
    adap = i2c_get_adapter(1);
    if (!adap) {
        E("E i2c adapter\n");
        return -ENODEV;
    } else
        client = i2c_new_device(adap, &hmc5883l_device);

    if (!client) {
        E("E i2c client\n");
        return -ENODEV;
    } else
        hmc5883l->client = client;

    D("INIT success!\n");

    return 0;
}

static void __exit hmc5883l_exit(void)
{
    i2c_del_driver(&hmc5883l_driver);
    if (hmc5883l->client)
        i2c_unregister_device(hmc5883l->client);
}

module_init(hmc5883l_init);
module_exit(hmc5883l_exit);
MODULE_AUTHOR("Kevin Liu");
MODULE_LICENSE("GPL");
