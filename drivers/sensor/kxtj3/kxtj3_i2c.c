/* 
 *  Kionix KXTJ3 3-axis accelerometer driver I2C support
 */

#define DT_DRV_COMPAT kionix_kxtj3

#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "kxtj3.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(kxtj3_i2c);

static int kxtj3_i2c_read_data(const struct device *dev, uint8_t reg_addr,
				               uint8_t *value, uint8_t len)
{
	const struct kxtj3_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->bus_cfg.i2c, reg_addr, value, len);
}

static int kxtj3_i2c_write_data(const struct device *dev, uint8_t reg_addr,
				                uint8_t *value, uint8_t len)
{
	const struct kxtj3_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->bus_cfg.i2c, reg_addr, value, len);
}

static int kxtj3_i2c_read_reg(const struct device *dev, uint8_t reg_addr,
				              uint8_t *value)
{
	const struct kxtj3_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->bus_cfg.i2c, reg_addr, value);
}

static int kxtj3_i2c_write_reg(const struct device *dev, 
	                           uint8_t reg_addr, uint8_t value)
{
	const struct kxtj3_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->bus_cfg.i2c, reg_addr, value);
}

static int kxtj3_i2c_update_reg(const struct device *dev,
	                            uint8_t reg_addr, uint8_t mask, uint8_t value)
{
	const struct kxtj3_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->bus_cfg.i2c, reg_addr, mask, value);
}

static const struct kxtj3_transfer_function kxtj3_i2c_transfer_fn = {
	.read_data  = kxtj3_i2c_read_data,
	.write_data = kxtj3_i2c_write_data,
	.read_reg   = kxtj3_i2c_read_reg,
	.write_reg  = kxtj3_i2c_write_reg,
	.update_reg = kxtj3_i2c_update_reg,
};

int kxtj3_i2c_init(const struct device *dev)
{
	struct kxtj3_data *data = dev->data;
	const struct kxtj3_config *cfg = dev->config;

	if (!device_is_ready(cfg->bus_cfg.i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	data->hw_tf = &kxtj3_i2c_transfer_fn;

	return 0;
}
