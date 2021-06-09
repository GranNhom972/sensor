/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SHT2XD_SHT2XD_H_
#define ZEPHYR_DRIVERS_SENSOR_SHT2XD_SHT2XD_H_

#include <device.h>
#include <kernel.h>
#include <drivers/gpio.h>



#define CMD_TRIGGER_T_H 0xE3 //0b11100011
#define CMD_TRIGGER_RH_H 0xE5 //0b11100101
#define CMD_TRIGGER_T_NH 0xF3 //0b11110011
#define CMD_TRIGGER_RH_NH 0xF5 //0b11110101
#define CMD_WRITE_USER_REGISTER 0xE6 //0b11100110
#define CMD_READ_USER_REGISTER 0xE7 //0b11100111 
#define CMD_SOFT_RESET 0xFE //0b11111110


#define SHT2XD_CHIP_ID  0b1000000
#define read_bit SHT2XD_CHIP_ID << 1
#define write_bit SHT2XD_CHIP_ID << 1 | 1

#define DURATION_MEASURE_00_TEMP 85
#define DURATION_MEASURE_00_HUMI 29


typedef struct device device;
typedef struct sht2xd_data sht2xd_data;
typedef struct sht2xd_bus_config sht2xd_bus_config;
typedef struct sht2xd_config sht2xd_config;

struct sht2xd_data 
{
	const device *dev;
	const device *bus;

	uint16_t t_sample;
	uint16_t rh_sample;

};

struct sht2xd_bus_config 
{
	uint16_t i2c_addr;
};

struct sht2xd_config 
{
	const char *bus_name;
	uint8_t base_address;

};
static inline uint8_t recuperer_adress_to_device(const device *dev)
{
	const sht2xd_config *dcp = dev->config;
	return dcp->base_address;
}

static inline const device *creer_pointer_to_bus(const device *dev)
{
	const sht2xd_data *ddp = dev->data;
	return ddp->bus;
}



#endif /* ZEPHYR_DRIVERS_SENSOR_sht2xd_sht2xd_H_ */
