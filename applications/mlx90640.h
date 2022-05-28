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

/* mlx90640 device structure */
struct mlx90640_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
};

#define MLX90640_I2C_ADDRESS                    0x33        // address pin A0,A1 low (GND), default for MLX90640
// #define MLX90640_I2C_ADDRESS                    (0x33 >> 1) // 7-bit address pin A0,A1 low (GND), default for MLX90640

#endif
