/*
 *  lm48100q amp driver (misc version)
 *
 *  Driver always uses inputs 1+2 (stereo).
 *  Driver sets volume for both channels to be identical.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>

#define LM48100Q_DRV_NAME	"lm48100q"
#define DRIVER_VERSION		"1.1"

/*
 * Defines
 */

#define LM48100Q_VOL1CTRL		0x60
#define LM48100Q_VOL2CTRL		0x80
#define LM48100Q_POWER_OFF		0x0C
#define LM48100Q_POWER_ON		0x1C

/*
 * Structs
 */

struct lm48100q_data {
	struct i2c_client *client;
	struct mutex update_lock;

	unsigned int power_state:1;
	unsigned int volumelvl;
};

/*
 * Management functions
 */

static int lm48100q_set_volumelvl(struct i2c_client *client, int volume)
{
	struct lm48100q_data *data = i2c_get_clientdata(client);
	int ret;

	ret = i2c_smbus_write_byte(client, LM48100Q_VOL1CTRL + volume);
	ret = i2c_smbus_write_byte(client, LM48100Q_VOL2CTRL + volume);

	data->volumelvl = volume;

	return ret;
}

static ssize_t lm48100q_store_volumelvl(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm48100q_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	if (val < 0 || val > 31)
		return -EINVAL;

	mutex_lock(&data->update_lock);
	ret = lm48100q_set_volumelvl(client, val);
	mutex_unlock(&data->update_lock);

	if (ret < 0)
		return ret;

	return count;
}

static ssize_t lm48100q_show_volumelvl(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lm48100q_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", data->volumelvl);
}

static DEVICE_ATTR(volumelvl, S_IWUSR | S_IRUGO,
		   lm48100q_show_volumelvl, lm48100q_store_volumelvl);

static int lm48100q_set_power_state(struct i2c_client *client, int state)
{
	struct lm48100q_data *data = i2c_get_clientdata(client);
	int ret;

	if (state == 0)
		ret = i2c_smbus_write_byte(client, LM48100Q_POWER_OFF);
	else {
		ret = i2c_smbus_write_byte(client, LM48100Q_POWER_ON);
	}

	data->power_state = state;

	return ret;
}

static ssize_t lm48100q_show_power_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lm48100q_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", data->power_state);
}

static ssize_t lm48100q_store_power_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm48100q_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	if (val < 0 || val > 1)
		return -EINVAL;

	mutex_lock(&data->update_lock);
	ret = lm48100q_set_power_state(client, val);
	mutex_unlock(&data->update_lock);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(power_state, S_IWUSR | S_IRUGO,
		   lm48100q_show_power_state, lm48100q_store_power_state);

static struct attribute *lm48100q_attributes[] = {
	&dev_attr_power_state.attr,
	&dev_attr_volumelvl.attr,
	NULL
};

static const struct attribute_group lm48100q_attr_group = {
	.attrs = lm48100q_attributes,
};

/*
 * Initialization function
 */

static int lm48100q_init_client(struct i2c_client *client)
{
	lm48100q_set_volumelvl(client,0);
	lm48100q_set_power_state(client, 0);

	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver lm48100q_driver;

static int lm48100q_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct lm48100q_data *data;
	int *opmode, err = 0;

	printk("lm48100q_probe here\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE
					    | I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct lm48100q_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->client = client;
	i2c_set_clientdata(client, data);

	/* Check platform data */
	opmode = client->dev.platform_data;

	mutex_init(&data->update_lock);

	/* Initialize the LM48100Q chip */
	err = lm48100q_init_client(client);
	if (err)
		goto exit_kfree;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &lm48100q_attr_group);
	if (err)
		goto exit_kfree;

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int lm48100q_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &lm48100q_attr_group);
	/* Power down the device */
	lm48100q_set_power_state(client, 0);
	i2c_set_clientdata(client, NULL);
	return 0;
}



static const struct i2c_device_id lm48100q_id[] = {
	{ "ti,lm48100q", 0 },
	{ }
};

static const struct of_device_id lm48100q_of_match[] = {
    { .compatible = "ti,lm48100q", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lm48100q_of_match);

static struct i2c_driver lm48100q_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= LM48100Q_DRV_NAME,
		.of_match_table = of_match_ptr(lm48100q_of_match),
	},
	.id_table = lm48100q_id,
	.probe	= lm48100q_probe,
	.remove	= lm48100q_remove,
};

module_i2c_driver(lm48100q_driver);

MODULE_AUTHOR("Chris Whittenburg <whittenburg@gmail.com>");
MODULE_DESCRIPTION("LM48100Q amplifier");
MODULE_LICENSE("GPL");

