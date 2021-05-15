/* ST Microelectronics LIS2DTW12 3-axis accelerometer driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lis2dtw12.pdf
 */

#define DT_DRV_COMPAT st_lis2dtw12

#include <string.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "lis2dtw12.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static uint16_t lis2dtw12_i2c_slave_addr = DT_INST_REG_ADDR(0);

#define HAL_I2C_TX_MAX_LEN (128)

LOG_MODULE_DECLARE(LIS2DTW12, CONFIG_SENSOR_LOG_LEVEL);

int hal_i2c_write(struct lis2dtw12_data *data, uint8_t addr, uint8_t reg, uint8_t *tx_buf, uint8_t tx_len)
	{
	if (!data->bus) {
		LOG_ERR("Device is not ready. Please init");
		return -1;
	}

	if (addr < 3 || addr > 0x77) {
		LOG_ERR("The address MUST be in range [0x04 .. 0x77]");
		return -1;
	}

	if (tx_len + 1 > HAL_I2C_TX_MAX_LEN) {
		LOG_ERR("Transmit len is out of range [0 to 128]");
		return -1;
	}

	uint8_t txbuf[HAL_I2C_TX_MAX_LEN] = {0x00};
	txbuf[0] = reg;
	memcpy(&txbuf[1], tx_buf, tx_len);
	if (i2c_write(data->bus, txbuf, tx_len + 1, addr) != 0) {
		return -1;
	}

	return 0;
}


int hal_i2c_read(struct lis2dtw12_data *data, uint8_t addr, uint8_t reg, uint8_t *rx_buf, uint8_t rx_len)
{
	if (!data->bus) {
	  LOG_ERR("Device is not ready. Please init");
	  return -1;
	}

	if (addr < 3 || addr > 0x77) {
	  LOG_ERR("The address MUST be in range [0x04 .. 0x77]");
	  return -1;
	}

	uint8_t txbuf[1] = {reg};
	if (i2c_write_read(data->bus, addr, txbuf, 1, rx_buf, rx_len) != 0) {
		return -1;
	}
  return 0;
}


// static int lis2dtw12_i2c_read(struct lis2dtw12_data *data, uint8_t reg_addr,
// 				 uint8_t *value, uint16_t len)
// {
// 	return i2c_burst_read(data->bus, lis2dtw12_i2c_slave_addr,
// 			      reg_addr, value, len);
// }

// static int lis2dtw12_i2c_write(struct lis2dtw12_data *data, uint8_t reg_addr,
// 				  uint8_t *value, uint16_t len)
// {
// 	return i2c_burst_write(data->bus, lis2dtw12_i2c_slave_addr, reg_addr, value, len);
// }

int32_t sensor_write_func(struct lis2dtw12_data *data, uint8_t reg, uint8_t *value, uint16_t len)
{
  return hal_i2c_write(data, lis2dtw12_i2c_slave_addr, reg, value, (uint16_t)len);
}

int32_t sensor_read_func(struct lis2dtw12_data *data, uint8_t reg, uint8_t *value, uint16_t len)
{
  return hal_i2c_read(data, lis2dtw12_i2c_slave_addr, reg, value, (uint16_t)len);
}


stmdev_ctx_t lis2dtw12_i2c_ctx = {
	.read_reg = (stmdev_read_ptr) sensor_read_func,
	.write_reg = (stmdev_write_ptr) sensor_write_func,
};

int lis2dtw12_i2c_init(const struct device *dev)
{
	struct lis2dtw12_data *data = dev->data;

	data->ctx = &lis2dtw12_i2c_ctx;
	data->ctx->handle = data;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
