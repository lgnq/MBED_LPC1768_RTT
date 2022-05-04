/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#ifndef __MLX90393_H__
#define __MLX90393_H__

#include <rtthread.h>
#include <stdint.h>

#define MLX90393_I2C_ADDRESS                    0x0C        // address pin A0,A1 low (GND), default for MLX90393
// #define MLX90393_I2C_ADDRESS                    (0x18 >> 1) // 7-bit address pin A0,A1 low (GND), default for MLX90393

enum cmd
{
    CMD_NOP               = 0x00,
    CMD_EXIT              = 0x80,
    CMD_START_BURST       = 0x10,
    CMD_WAKE_ON_CHANGE    = 0x20,
    CMD_START_MEASUREMENT = 0x30,
    CMD_READ_MEASUREMENT  = 0x40,
    CMD_READ_REGISTER     = 0x50,
    CMD_WRITE_REGISTER    = 0x60,
    CMD_MEMORY_RECALL     = 0xd0,
    CMD_MEMORY_STORE      = 0xe0,
    CMD_RESET             = 0xf0
};

enum
{
    Z_FLAG = 0x8,
    Y_FLAG = 0x4,
    X_FLAG = 0x2,
    T_FLAG = 0x1
} axis_flag_t;

/* Accelerometer full scale range */
enum mlx90393_accel_range
{
    MPU6XXX_ACCEL_RANGE_2G  = 0, // ±2G
    MPU6XXX_ACCEL_RANGE_4G  = 1, // ±4G
    MPU6XXX_ACCEL_RANGE_8G  = 2, // ±8G
    MPU6XXX_ACCEL_RANGE_16G = 3  // ±16G
};

/* sleep mode parameters */
enum mlx90393_sleep
{
    MPU6XXX_SLEEP_DISABLE = 0,
    MPU6XXX_SLEEP_ENABLE  = 1
};

/* Supported configuration items */
enum mlx90393_cmd
{
    MPU6XXX_GYRO_RANGE,  /* Gyroscope full scale range */
    MPU6XXX_ACCEL_RANGE, /* Accelerometer full scale range */
    MPU6XXX_DLPF_CONFIG, /* Digital Low Pass Filter */
    MPU6XXX_SAMPLE_RATE, /* Sample Rate —— 16-bit unsigned value.
                            Sample Rate = [1000 -  4]HZ when dlpf is enable
                            Sample Rate = [8000 - 32]HZ when dlpf is disable */
    MPU6XXX_SLEEP        /* Sleep mode */
};

/** Gain settings for CONF1 register. */
typedef enum mlx90393_gain 
{
    MLX90393_GAIN_5X = (0x00),
    MLX90393_GAIN_4X,
    MLX90393_GAIN_3X,
    MLX90393_GAIN_2_5X,
    MLX90393_GAIN_2X,
    MLX90393_GAIN_1_67X,
    MLX90393_GAIN_1_33X,
    MLX90393_GAIN_1X
} mlx90393_gain_t;

union mlx90393_status
{
    struct
    {
    /** D1-D0: indicates the number of bytes (2D[1:0]) to follow the status byte after a read measurement
     * or a read register command has been sent.
     */
    uint8_t d0 : 1;
    /** D1-D0: indicates the number of bytes (2D[1:0]) to follow the status byte after a read measurement
     * or a read register command has been sent.
     */
    uint8_t d1 : 1;

    /** RS: indicates that the device has been reset successfully by a reset command.
     */
    uint8_t rs : 1;

    /** SED: indicates that a single bit error has been corrected by the NVRAM
     */
    uint8_t sed : 1;

    /** ERROR: indicates an error.
     * Can be set when reading out a measurement while the measurement is not yet completed or
     * when reading out the same measurement twice.
     */
    uint8_t error : 1;

    /** SM_mode: if set, the IC is executing a measurement sequence in polling mode.
     * It can be initiated by a SM command or a pulse on the TRIG input.
     */
    uint8_t sm_mode : 1;

    /** WOC_mode: if set, the IC is in wake-up-on-change mode.
     */
    uint8_t woc_mode : 1;

    /** Burst_mode: if set, the IC is working in burst mode.
     */
    uint8_t burst_mode : 1;
    };

    uint8_t byte_val;
};

union mlx90393_register0
{
    uint16_t word_val;

    struct
    {
        uint8_t hallconf            : 4;
        uint8_t gain_sel            : 3;
        uint8_t z_series            : 1;
        uint8_t bist                : 1;
        uint8_t ana_reserved_low    : 7;
    };
};

union mlx90393_register1
{
    uint16_t word_val;

    struct
    {
        uint8_t burst_data_rate     : 6;
        uint8_t burst_sel           : 4;
        uint8_t tcmp_en             : 1;
        uint8_t ext_trg             : 1;
        uint8_t woc_diff            : 1;
        uint8_t comm_mode           : 2;
        uint8_t trig_int            : 1;
    };
};

union mlx90393_register2
{
    uint16_t word_val;

    struct
    {
        uint8_t osr         : 2;
        uint8_t dig_filt    : 3;
        uint8_t res_x       : 2;
        uint8_t res_y       : 2;
        uint8_t res_z       : 2;
        uint8_t osr2        : 2;
        uint8_t reserved    : 3;
    };
};

union mlx90393_register3
{
    uint16_t word_val;

    struct
    {
        uint8_t sens_tc_lt  : 8;
        uint8_t sens_tc_ht  : 8;
    };
};

/* 3-axis data structure */
struct mlx90393_txyz
{
    rt_int16_t t;
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};

/* mlx90393 config structure */
struct mlx90393_config
{
    rt_uint16_t accel_range;
    rt_uint16_t gyro_range;
};

/* mlx90393 device structure */
struct mlx90393_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    struct mlx90393_config config;
};

/**
 * This function initialize the mlx90393 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct mlx90393_device *mlx90393_init(const char *dev_name, rt_uint8_t param);

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx90393_deinit(struct mlx90393_device *dev);

/**
 * This function set mlx90393 parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param Configuration item parameter
 *
 * @return the setting status, RT_EOK reprensents  setting the parameter successfully.
 */
rt_err_t mlx90393_set_param(struct mlx90393_device *dev, enum mlx90393_cmd cmd, rt_uint16_t param);

/**
* This function gets the data of the mps, unit: mg
 *
 * @param dev the pointer of device driver structure
 * @param mps the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mlx90393_get_mps(struct mlx90393_device *dev, struct mlx90393_3axes *accel);

/**
* This function gets the data of the gyroscope, unit: deg/10s
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mlx90393_get_gyro(struct mlx90393_device *dev, struct mlx90393_3axes *gyro);

/**
 * This function gets the data of the temperature, unit: Centigrade
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mlx90393_get_temp(struct mlx90393_device *dev, float *temp);

/**
* This function sets the offset of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents setting the offsets successfully.
 */
rt_err_t mlx90393_set_accel_offset(struct mlx90393_device *dev, struct mlx90393_3axes *offset);

/**
* This function gets the offset of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents reading the offsets successfully.
 */
rt_err_t mlx90393_get_accel_offset(struct mlx90393_device *dev, struct mlx90393_3axes *offset);

/**
* This function sets the offset of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents setting the offsets successfully.
 */
rt_err_t mlx90393_set_gyro_offset(struct mlx90393_device *dev, struct mlx90393_3axes *offset);

/**
* This function gets the offset of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents reading the offsets successfully.
 */
rt_err_t mlx90393_get_gyro_offset(struct mlx90393_device *dev, struct mlx90393_3axes *offset);

rt_err_t mlx90393_nop(struct mlx90393_device *dev);
rt_err_t mlx90393_exit(struct mlx90393_device *dev);
rt_err_t mlx90393_reset(struct mlx90393_device *dev);

rt_int8_t mlx90393_get_gain_sel(struct mlx90393_device *dev, mlx90393_gain_t *gain);

#endif
