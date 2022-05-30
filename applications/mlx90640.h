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

#define MLX90640_DEBUG

#define MLX90640_I2C_ADDRESS                    0x33        // default I2C address of MLX90640 is 0x33

#define EEPROM_START_ADDR                       0x2400
#define EEPROM_LENGTH                           832

#define STATUS_REGISTER_ADDR                    0x8000
#define CONTROL_REGISTER_1_ADDR                 0x800D
#define I2C_CONFIGURATION_REGISTER              0x800F

enum mlx90640_resolution
{
    ADC_SET_TO_16_BIT_RESOLUTION = 0,
    ADC_SET_TO_17_BIT_RESOLUTION = 1,
    ADC_SET_TO_18_BIT_RESOLUTION = 2,   //default
    ADC_SET_TO_19_BIT_RESOLUTION = 3
};

enum mlx90640_refresh_rate
{
    IR_REFRESH_RATE_0_5_HZ  = 0,
    IR_REFRESH_RATE_1_HZ    = 1,
    IR_REFRESH_RATE_2_HZ    = 2,    //default
    IR_REFRESH_RATE_4_HZ    = 3,
    IR_REFRESH_RATE_8_HZ    = 4,
    IR_REFRESH_RATE_16_HZ   = 5,
    IR_REFRESH_RATE_32_HZ   = 6,
    IR_REFRESH_RATE_64_HZ   = 7
};

enum mlx90640_reading_pattern
{
    INTERLEAVED_MODE  = 0,
    CHESS_PATTERN     = 1,  //default
};

union mlx90640_control_register1
{
    rt_uint16_t word_val;

    struct
    {
        rt_uint8_t enable_subpages_mode                 : 1;
        rt_uint8_t reserved1                            : 1;
        rt_uint8_t enable_data_hold                     : 1;
        rt_uint8_t enable_subpages_repeat               : 1;
        rt_uint8_t select_subpage                       : 3;
        enum mlx90640_refresh_rate refresh_rate         : 3;
        enum mlx90640_resolution resolution             : 2;
        enum mlx90640_reading_pattern reading_pattern   : 1;
        rt_uint8_t reserved2                            : 3;
    };
};

/* mlx90640 device structure */
struct mlx90640_device
{
    rt_device_t bus;
    rt_uint16_t id[3];
    rt_uint8_t i2c_addr;

    rt_int16_t kVdd;
    rt_int16_t vdd25;
    float KvPTAT;
    float KtPTAT;
    rt_uint16_t vPTAT25;
    float alphaPTAT;
    rt_int16_t gainEE;
    float tgc;
    float cpKv;
    float cpKta;
    rt_uint8_t resolutionEE;
    rt_uint8_t calibrationModeEE;
    float KsTa;
    float ksTo[5];
    rt_int16_t ct[5];
    rt_uint16_t alpha[768];
    rt_uint8_t alphaScale;
    rt_int16_t offset[768];
    rt_int8_t kta[768];
    rt_uint8_t ktaScale;
    rt_int8_t kv[768];
    rt_uint8_t kvScale;
    float cpAlpha[2];
    rt_int16_t cpOffset[2];
    float ilChessC[3];
    rt_uint16_t brokenPixels[5];
    rt_uint16_t outlierPixels[5];    

    enum mlx90640_resolution resolution;
    enum mlx90640_refresh_rate refresh_rate;
    enum mlx90640_reading_pattern reading_pattern;    
};

#endif
