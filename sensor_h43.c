#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/delay.h>

#include "log.h"

#define GPIO1_16 48
#define I2C_ADAPTER_ID 2

#define SENSOR_ID_STRING "H43"
#define SENSOR_NAME "hmc5883l-i2c"
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
    #define DATA_READY      0x1

#define HMC5883L_IDENTIFY_REG_A  0x0A
#define HMC5883L_IDENTIFY_REG_B  0x0B
#define HMC5883L_IDENTIFY_REG_C  0x0C

struct sensor_hmc5883l {
    struct mutex lock;
    struct i2c_client *client;
    struct work_struct work;
    struct completion wait;
    bool is_self_test;
    u16 x_axis;
    u16 y_axis;
    u16 z_axis;
    u8 sample;
    u8 out_rate;
    u8 mesura;
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

static s32 hmc5883l_write_regA(struct i2c_client *client)
{
    u8 val;
    mutex_lock(&hmc5883l->lock);
    val = (hmc5883l->sample << SAMPLE_AVER_OFFSET)
        | (hmc5883l->out_rate << DATA_OUT_RATE_OFFSET)
        | hmc5883l->mesura;
    mutex_unlock(&hmc5883l->lock);
    return hmc5883l_write_byte(client, HMC5883L_CONFIG_REG_A, val);
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
    mutex_lock(&hmc5883l->lock);
    hmc5883l->mode = mode;
    mutex_unlock(&hmc5883l->lock);
    mode = mode | (0x0 << 7);

    result = hmc5883l_write_byte(client, HMC5883L_MODE_REG, mode);
    return (result > 0? -EBUSY : result);
}

static u8 hmc5883l_get_mode(struct i2c_client *client)
{
    return hmc5883l->mode;
    //return hmc5883l_read_byte(client, HMC5883L_MODE_REG) & HMC5883L_MODE_REG;
}

static s32 hmc5883l_set_sample_average(struct i2c_client *client, u8 sample)
{
    sample = sample & SAMPLE_AVER;
    if (sample > 3 || sample < 0)
        return -EINVAL;
    mutex_lock(&hmc5883l->lock);
    hmc5883l->sample = sample;
    mutex_unlock(&hmc5883l->lock);
    return hmc5883l_write_regA(client);
}

static u8 hmc5883l_get_sample_average(struct i2c_client *client)
{
    return hmc5883l->sample;
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
    mutex_lock(&hmc5883l->lock);
    hmc5883l->gain = gain;
    mutex_unlock(&hmc5883l->lock);

    result = hmc5883l_write_byte(client, HMC5883L_CONFIG_REG_B, gain << GAIN_SETTING_OFFSET);
    return (result > 0? -EBUSY : result);
}

static u8 hmc5883l_get_gain(struct i2c_client *client)
{
    return hmc5883l->gain;
    //return hmc5883l_read_byte(client, HMC5883L_CONFIG_REG_B) >> GAIN_SETTING_OFFSET & GAIN_SETTING;
}

static s32 hmc5883l_set_mesura(struct i2c_client *client, u8 mesura)
{
    mesura = mesura & MESURE_SETTING;
    if (mesura > MAX_MESURA)
        return -EINVAL;

    mutex_lock(&hmc5883l->lock);
    hmc5883l->mesura = mesura;
    mutex_unlock(&hmc5883l->lock);
    return hmc5883l_write_regA(client);
}

static u8 hmc5883l_get_mesura(struct i2c_client *client)
{
    /*
    s32 result;
    result = hmc5883l_read_byte(client, HMC5883L_CONFIG_REG_A);
    return (result & MESURE_SETTING);
    */
    return hmc5883l->mesura;
}

static s32 hmc5883l_set_data_out_rate(struct i2c_client *client,
        u8 rate)
{
    rate = rate & DATA_OUT_RATE;
    if (rate >= 7 || rate < 0) {
        E("Invalid data output rate\n");
        return -EINVAL;
    }
    mutex_lock(&hmc5883l->lock);
    hmc5883l->out_rate = rate;
    mutex_unlock(&hmc5883l->lock);
    return hmc5883l_write_regA(client);
}

static u8 hmc5883l_get_data_out_rate(struct i2c_client *client)
{
    return hmc5883l->out_rate;
}

static s32 hmc5883l_get_version(struct i2c_client *client)
{
    s32 value_A = hmc5883l_read_byte(client, HMC5883L_IDENTIFY_REG_A);
    s32 value_B = hmc5883l_read_byte(client, HMC5883L_IDENTIFY_REG_B);
    s32 value_C = hmc5883l_read_byte(client, HMC5883L_IDENTIFY_REG_C);

    D("A=0x%X B=0x%X C=0x%X\n", value_A, value_B, value_C);
    s32 version = value_A & 0xff | value_B & 0xff << 8
        | value_C & 0xff << 16;
    return version;
}

static bool hmc5883l_is_data_ready(struct i2c_client *client)
{
    s32 result;
    result = hmc5883l_read_byte(client, HMC5883L_STATUS_REG);
    D("hmc5883l_is_data_ready: value = 0x%X\n", result & DATA_READY);
    if (result & DATA_READY) {
        D("D1111111\n");
        return true;
    } else {
        D("DDDDDDDDDDDD\n");
        return false;
    }
}

static bool hmc5883l_is_reg_locked(struct i2c_client *client)
{
    s32 result;
    result = hmc5883l_read_byte(client, HMC5883L_STATUS_REG);
    D("hmc5883l_is_reg_locked: value = 0x%X\n", result >> DATA_LOCK_OFFSET & DATA_LOCK);
    if (result >> DATA_LOCK_OFFSET & DATA_LOCK)
        return true;
    else
        return false;
}

static irqreturn_t hmc5883l_interrupt(int irq, void *dev_id)
{
    if (hmc5883l->is_self_test) {
        complete(&hmc5883l->wait);
        goto out;
    }
    schedule_work(&hmc5883l->work);
out:
    return IRQ_HANDLED;
}

static void hmc5883l_work_queue(struct work_struct *work)
{
    struct i2c_client *client = hmc5883l->client;

    hmc5883l->x_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG) << 8
        | hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 1);
    hmc5883l->y_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 2) << 8
        | hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 3);
    hmc5883l->z_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 4) << 8
        | hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 5);
    D("0x%04X 0x%04X 0x%04X\n", hmc5883l->x_axis,
            hmc5883l->y_axis, hmc5883l->z_axis);
#if 0
    if (hmc5883l->x_axis > 0xF800 || hmc5883l->y_axis > 0xF800
            || hmc5883l->z_axis > 0xF800) {
        if (hmc5883l->gain < 8) {
            hmc5883l->gain++;
            D("Gain too large, gain => %d\n", hmc5883l->gain);
            hmc5883l_set_gain(client, hmc5883l->gain);
        }
    } else if (hmc5883l->x_axis < 0x07FF || hmc5883l->y_axis < 0x07FF
            || hmc5883l->z_axis < 0x07FF) {
        if (hmc5883l->gain > 0) {
            hmc5883l->gain--;
            D("Gain too little, gain => %d\n", hmc5883l->gain);
            hmc5883l_set_gain(client, hmc5883l->gain);
        }
    }
#endif
}

static void do_self_test(struct i2c_client *client)
{
    unsigned long timeout;
    int i = 0;

    hmc5883l->is_self_test = true;
    hmc5883l_write_byte(client, 0x00, 0x71);
    hmc5883l_write_byte(client, 0x01, 0xA0);
    hmc5883l_write_byte(client, 0x02, 0x00);

    for (i = 0; i < 3; i++) {
        timeout = wait_for_completion_timeout(&hmc5883l->wait, HZ);
        if (timeout <= 0) {
            E("self test timeout\n");
            return;
        }

        hmc5883l->x_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG) << 8
            | hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 1);
        hmc5883l->y_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 2) << 8
            | hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 3);
        hmc5883l->z_axis = hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 4) << 8
            | hmc5883l_read_byte(client, HMC5883L_DATA_OUT_REG + 5);
        D("0x%04X 0x%04X 0x%04X\n", hmc5883l->x_axis,
                hmc5883l->y_axis, hmc5883l->z_axis);
        hmc5883l_write_byte(client, 0x03, 0x00);
    }

    hmc5883l->is_self_test = false;
    hmc5883l_set_mode(client, IDLE_MODE);
}

static int hmc5883l_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    i2c_set_clientdata(client, hmc5883l);
    ret = request_irq(client->irq, hmc5883l_interrupt,
            IRQF_TRIGGER_RISING, "hmc5883l-i2c", NULL);
    if (ret)
        E("IRQ request met err\n");

    do_self_test(client);
    hmc5883l->mesura = NORMAL_MESURA;
    hmc5883l->gain = 0x01;
    hmc5883l->mode = CONTINOUS_MODE;
    hmc5883l->out_rate = 0x04;
    hmc5883l->sample = 0x03;

    hmc5883l_set_mode(client, hmc5883l->mode);
    hmc5883l_set_gain(client, hmc5883l->gain);
    hmc5883l_set_data_out_rate(client, hmc5883l->out_rate);
    hmc5883l_set_mesura(client, hmc5883l->mesura);
    hmc5883l_set_sample_average(client, hmc5883l->sample);
    hmc5883l_get_version(client);
    return 0;
}

static int hmc5883l_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id hmc5883l_id[] = {
    {"hmc5883l-i2c", 0},
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
    I2C_BOARD_INFO("hmc5883l-i2c", SENSOR_SLAVE_ADDRESS),
};

static int __init hmc5883l_init(void)
{
    struct i2c_adapter *adap;
    struct i2c_client *client;
    int adap_num = 1;

    hmc5883l = kzalloc(sizeof(*hmc5883l), GFP_KERNEL);
    if (!hmc5883l)
        return -ENOMEM;
    mutex_init(&hmc5883l->lock);
    INIT_WORK(&hmc5883l->work, hmc5883l_work_queue);
    init_completion(&hmc5883l->wait);
    hmc5883l->is_self_test = false;

    i2c_add_driver(&hmc5883l_driver);

    adap = i2c_get_adapter(adap_num);
    if (!adap) {
        E("E i2c adapter %d\n", adap_num);
        return -ENODEV;
    } else {
        D("Get Adapter %d\n", adap_num);
        hmc5883l_device.irq = gpio_to_irq(DRDY_INT);
        client = i2c_new_device(adap, &hmc5883l_device);
    }

    if (!client) {
        E("E i2c client %s @ %0x%02X\n", hmc5883l_device.type,
                hmc5883l_device.addr);
        return -ENODEV;
    } else {
        D("Client OK!\n");
        hmc5883l->client = client;
    }
    D("INIT success!\n");
    i2c_put_adapter(adap);

    return 0;
}

static void __exit hmc5883l_exit(void)
{
    i2c_del_driver(&hmc5883l_driver);
    free_irq(hmc5883l->client->irq, NULL);
    if (hmc5883l->client)
        i2c_unregister_device(hmc5883l->client);
    D("Module removed\n");
}

module_init(hmc5883l_init);
module_exit(hmc5883l_exit);
MODULE_AUTHOR("Kevin Liu");
MODULE_LICENSE("GPL");
