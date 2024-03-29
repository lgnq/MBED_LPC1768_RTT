/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-11     lgnq         The first version for LPC17xx
 */

#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#include "LPC17xx.h"

#ifdef RT_USING_I2C_BITOPS
#ifdef RT_USING_PIN
/* lpc17xx config class */
struct lpc17xx_soft_i2c_config
{
    rt_uint8_t scl;
    rt_uint8_t sda;
    const char *bus_name;
};
/* lpc17xx i2c dirver class */

struct lpc17xx_i2c
{
    struct rt_i2c_bit_ops ops;
    struct rt_i2c_bus_device i2c2_bus;
};

/* register I2C0: SCL/P0_28 SDA/P0_27 */
/* register I2C1: SCL/P0_01 SDA/P0_00 */
/* register I2C2: SCL/P0_11 SDA/P0_10 */

#ifdef BSP_USING_I2C0
#define I2C0_BUS_CONFIG                     \
    {                                       \
        .scl = 28,                          \
        .sda = 27,                          \
        .bus_name = "i2c0",                 \
    }
#endif

#ifdef BSP_USING_I2C1
#define I2C1_BUS_CONFIG                     \
    {                                       \
        .scl = 1,                           \
        .sda = 0,                           \
        .bus_name = "i2c1",                 \
    }
#endif
    
#ifdef BSP_USING_I2C2
#define I2C2_BUS_CONFIG                     \
    {                                       \
        .scl = 11,                          \
        .sda = 10,                          \
        .bus_name = "i2c2",                 \
    }
#endif

#else
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#endif
#else
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#endif

int rt_hw_i2c_init(void);

#endif
