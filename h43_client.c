#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

MODULE_DESCRIPTION("HMC5883L sensor i2c bus client module");
MODULE_AUTHOR("Kevin Liu (airk908@gmail.com)");
MODULE_LICENSE("GPL");

static struct i2c_board_info hmc5883l_info = {
    I2C_BOARD_INFO("hmc5883l-i2c", 0x1e)
};

static int __init hmc5883l_client_init(void)
{
    struct i2c_adapter *adap;
    struct i2c_client *client;
    int adap_nr = 1; // 0 doesn't work

    adap = i2c_get_adapter(adap_nr);
    if (!adap)
        return -ENODEV;

    hmc5883l_info.irq = gpio_to_irq(48);
    client = i2c_new_device(adap, &hmc5883l_info);
    if (!client) {
        i2c_put_adapter(adap);
        return -ENODEV;
    }

    return 0;
}

static void __exit hmc5883l_client_exit(void)
{
    return;
}

module_init(hmc5883l_client_init);
module_exit(hmc5883l_client_exit);
