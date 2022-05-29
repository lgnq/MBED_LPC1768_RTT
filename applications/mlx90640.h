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

enum mlx90640_resolution
{
    ADC_SET_TO_16_BIT_RESOLUTION = 0,
    ADC_SET_TO_17_BIT_RESOLUTION = 1,
    ADC_SET_TO_18_BIT_RESOLUTION = 2,   //default
    ADC_SET_TO_19_BIT_RESOLUTION = 3
};

union mlx90640_control_register1
{
    rt_uint16_t word_val;

    struct
    {
        rt_uint8_t enable_subpages_mode         : 1;
        rt_uint8_t reserved1                    : 1;
        rt_uint8_t enable_data_hold             : 1;
        rt_uint8_t enable_subpages_repeat       : 1;
        rt_uint8_t select_subpage               : 3;
        rt_uint8_t refresh_rate                 : 3;
        enum mlx90640_resolution resolution     : 2;
        rt_uint8_t reading_pattern              : 1;
        rt_uint8_t reserved2                    : 3;
    };
};

#endif
