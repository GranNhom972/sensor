/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */




#include <device.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>


#include "sht2xd.h"
#include <logging/log.h>

#define DT_DRV_COMPAT sht2xd
LOG_MODULE_REGISTER(SHT2XD, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "sht2xdXD driver n'est pas autorisé"
#endif


void error_message(const char *message);

//#####################################################################
/*
 * CRC algorithm parameters were taken from the
 * "Checksum Calculation" section of the datasheet.
 */

static uint8_t crc_calculation(uint16_t value)
{
	uint8_t buf[2] = { value >> 8, value & 0xFF };
	uint8_t crc = 0;	
	uint8_t byteCtr;
	uint16_t polynom = 0x131;  //P(x)=x^8+x^5+x^4+1 = 100110001

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < 2; ++byteCtr)
	{ 
			crc ^= (buf[byteCtr]);
			for (uint8_t bit = 8; bit > 0; --bit)
			{ 
				if (crc & 0x80) crc = (crc << 1) ^ polynom;
				else crc = (crc << 1);
			}
	}
	
	return crc;
}

//#####################################################################
static int sht2xd_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	sht2xd_data *data = dev->data;
	const device *i2c =creer_pointer_to_bus(dev);
	uint8_t address = recuperer_adress_to_device(dev);
	uint8_t rx_buf_temp[3];
	uint8_t rx_buf_humi[3];
	uint16_t t_sample, rh_sample;
	uint8_t com_write_t=CMD_TRIGGER_T_H;
	uint8_t com_write_rh=CMD_TRIGGER_RH_H;
	/*DEBUT TEMPERATURE*/
	if(chan==SENSOR_CHAN_AMBIENT_TEMP)
	{	
		//requete lecture Temperature
		if(i2c_write(i2c,&com_write_t, sizeof(com_write_t), address)<0)
		{
			error_message("cmd temp\n");
			return -EIO;
		}
		k_sleep(K_MSEC(DURATION_MEASURE_00_TEMP));

		//lecture donnée Temperature
		if(i2c_read(i2c, rx_buf_temp, sizeof(rx_buf_temp), address)<0)
		{
			error_message("rx temp\n");
			return -EIO;
		}
		t_sample=(rx_buf_temp[0]<<8)|(rx_buf_temp[1]);
		if (crc_calculation(t_sample) != rx_buf_temp[2])
		{
			error_message("crctemp\n");
			return -EIO;
		}
		data->t_sample = t_sample;
		
	}
	/*FIN TEMPERATURE*/

	/*DEBUUT HUMIDITE*/
	else if(chan==SENSOR_CHAN_HUMIDITY)
	{
		//requete lecture humidité
		if(i2c_write(i2c, &com_write_rh, sizeof(com_write_rh), address)<0)
		{
			error_message("erreur\n");
			return -EIO;
		}
		k_sleep(K_MSEC(DURATION_MEASURE_00_HUMI));

		//lecture donnée humidité
		if(i2c_read(i2c, rx_buf_humi, sizeof(rx_buf_humi), address)<0)
		{
			error_message("error\n");
			return -EIO;
		}
		rh_sample=(rx_buf_humi[0]<<8)|(rx_buf_humi[1]);
		if (crc_calculation(rh_sample) != rx_buf_humi[2])
		{
			error_message("Checksum faux\n");
			return -EIO;
		}
		data->rh_sample = rh_sample;
	}
	/*FIN HUMIDITE*/

	return 0;
}
//#####################################################################


//#####################################################################
static int sht2xd_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	const sht2xd_data *data = dev->data;
	uint64_t tmp;

	/*
	 * See datasheet "Conversion of Signal Output" section
	 * for more details on processing sample data.
	 */
	if (chan == SENSOR_CHAN_AMBIENT_TEMP) 
	{
		/* val = -45 + 175 * sample / (2^16 -1) */
		tmp = (uint64_t)data->t_sample * 175U;
		val->val1 = (int32_t)(tmp / 0xFFFF) - 45;
		val->val2 = ((tmp % 0xFFFF) * 1000000U) / 0xFFFF;
	} 
	else if (chan == SENSOR_CHAN_HUMIDITY) 
	{
		/* val = 100 * sample / (2^16 -1) */
		uint32_t tmp2 = (uint32_t)data->rh_sample * 100U;
		val->val1 = tmp2 / 0xFFFF;
		/* x * 100000 / 65536 == x * 15625 / 1024 */
		val->val2 = (tmp2 % 0xFFFF) * 15625U / 1024;
	}
	else 
	{
		return -ENOTSUP;
	}

	return 0;
}
//#####################################################################

static const struct sensor_driver_api sht2xd_driver_api = 
{
	.sample_fetch = sht2xd_sample_fetch,
	.channel_get = sht2xd_channel_get,
};

//#####################################################################
static int sht2xd_init(const struct device *dev)
{
	struct sht2xd_data *data = dev->data;

	const struct sht2xd_config *cfg = dev->config;
	const struct device *i2c = device_get_binding(cfg->bus_name);
	if (i2c == NULL) 
	{
		LOG_DBG("Failed to get pointer to %s device!", cfg->bus_name);
		return -EINVAL;
	}
	data->bus = i2c;

	if (!cfg->base_address) 
	{
		LOG_DBG("No I2C address");
		return -EINVAL;
	}
	data->dev = dev;
	return 0;
}
//#####################################################################

struct sht2xd_data sht2xd0_driver;
static const struct sht2xd_config sht2xd0_cfg = 
{
		.bus_name = DT_INST_BUS_LABEL(0),

		.base_address = DT_INST_REG_ADDR(0),
	
};

DEVICE_DT_INST_DEFINE(0, sht2xd_init, device_pm_control_nop, &sht2xd0_driver, &sht2xd0_cfg, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &sht2xd_driver_api);
