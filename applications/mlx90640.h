/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-25     lgnq         the first version
 */

#ifndef __MLX90640_H__
#define __MLX90640_H__

#include <rtthread.h>

#define MLX90640_I2C_ADDRESS                    0x33        // default I2C address of MLX90640 is 0x33

#define STATUS_REGISTER_ADDR                    0x8000
#define CONTROL_REGISTER_1_ADDR                 0x800D
#define I2C_CONFIGURATION_REGISTER              0x800F

/* mlx90640 device structure */
struct mlx90640_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
};

#endif
