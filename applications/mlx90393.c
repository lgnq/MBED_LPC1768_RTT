/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "mlx90393.h"

#include <string.h>
#include <stdlib.h>

rt_err_t mlx90393_transfer(struct mlx90393_device *dev, rt_uint8_t *send_buf, rt_uint8_t *recv_buf, rt_uint8_t send_len, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;
    union mlx90393_status status;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs[2];

        msgs[0].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = send_buf;         /* Write data pointer */
        msgs[0].len   = send_len;         /* Number of bytes write */

        msgs[1].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = recv_buf;         /* Read data pointer */
        msgs[1].len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            status.byte_val = recv_buf[0];
            
            rt_kprintf("status = 0x%x\r\n", status.byte_val);
            rt_kprintf("[BIT7] BURST_MODE = 0x%x - MLX90393 works in Burst mode\r\n", status.burst_mode);
            rt_kprintf("[BIT6] WOC_MODE   = 0x%x - MLX90393 works in Wake On Change mode\r\n", status.woc_mode);
            rt_kprintf("[BIT5] SM_MODE    = 0x%x - MLX90393 works in Single measurement mode\r\n", status.sm_mode);
            rt_kprintf("[BIT4] ERROR      = 0x%x - ECC_ERROR or command is rejected\r\n", status.error);
            rt_kprintf("[BIT3] SED        = 0x%x - a bit error in the non-volatile memory has been corrected\r\n", status.sed);
            rt_kprintf("[BIT2] RS         = 0x%x - Reset bit\r\n", status.rs);
            rt_kprintf("[BIT1] D1         = 0x%x - The number of response bytes correspond to 2*D[1:0]+2\r\n", status.d1);
            rt_kprintf("[BIT0] D0         = 0x%x - The number of response bytes correspond to 2*D[1:0]+2\r\n\r\n", status.d0);

            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
#ifdef RT_USING_SPI
        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, send_buf, send_len, recv_buf, recv_len);
#endif
    }
    else
    {
        
    }

    return res;
}

rt_err_t mlx90393_nop(struct mlx90393_device *dev)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = CMD_NOP;

    return(mlx90393_transfer(dev, send_buf, recv_buf, 1, 1));
}

rt_err_t mlx90393_exit(struct mlx90393_device *dev)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = CMD_EXIT;

    return(mlx90393_transfer(dev, send_buf, recv_buf, 1, 1));    
}

rt_err_t mlx90393_memory_recall(struct mlx90393_device *dev)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = CMD_MEMORY_RECALL;

    return(mlx90393_transfer(dev, send_buf, recv_buf, 1, 1));    
}

rt_err_t mlx90393_memory_store(struct mlx90393_device *dev)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = CMD_MEMORY_STORE;

    return(mlx90393_transfer(dev, send_buf, recv_buf, 1, 1));     
}

rt_err_t mlx90393_reset(struct mlx90393_device *dev)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = CMD_RESET;

    return(mlx90393_transfer(dev, send_buf, recv_buf, 1, 1));     
}

rt_err_t mlx90393_start_burst(struct mlx90393_device *dev, rt_int8_t zyxt)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = (CMD_START_BURST)|(zyxt);

    return(mlx90393_transfer(dev, send_buf, recv_buf, 1, 1));
}

rt_err_t mlx90393_wake_on_change(struct mlx90393_device *dev, rt_int8_t zyxt)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = (CMD_WAKE_ON_CHANGE)|(zyxt);

    return(mlx90393_transfer(dev, send_buf, recv_buf, 1, 1));
}

rt_err_t mlx90393_start_measurement(struct mlx90393_device *dev, rt_int8_t zyxt)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = (CMD_START_MEASUREMENT)|(zyxt);

    return(mlx90393_transfer(dev, send_buf, recv_buf, 1, 1));
}

/**
 * This function reads the value of register for mlx90393
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90393
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
static rt_err_t mlx90393_read_reg(struct mlx90393_device *dev, rt_uint8_t reg, rt_uint16_t *val)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t send_buf[10];
    rt_uint8_t recv_buf[3];

    send_buf[0] = CMD_READ_REGISTER;
    send_buf[1] = reg << 2;

    res = mlx90393_transfer(dev, send_buf, recv_buf, 2, 3);
    if (res == RT_EOK)
    {
        *val = ((uint16_t)recv_buf[1])<<8 | recv_buf[2];
    }

    return res;
}

/**
 * This function writes the value of the register for mlx90393
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90393
 * @param val value to write
 *
 * @return the writing status, RT_EOK represents writing the value of the register successfully.
 */
static rt_err_t mlx90393_write_reg(struct mlx90393_device *dev, rt_uint8_t reg, rt_uint16_t val)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t recv_buf[3];
    rt_uint8_t send_buf[] =
    {
        CMD_WRITE_REGISTER,
        (val&0xFF00) >> 8,
        val&0x00FF,
        reg << 2
    };

    res = mlx90393_transfer(dev, send_buf, recv_buf, 4, 1);

    return res;
}

int count_set_bits(int N)
{
    int result = 0;

    while (N)
    {
        result++;
        N &=N-1;
    }

    return result;
}

rt_err_t mlx90393_read_measurement(struct mlx90393_device *dev, rt_int8_t zyxt, struct mlx90393_txyz *txyz)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[10];

    send_buf[0] = (CMD_READ_MEASUREMENT)|(zyxt);
    for (int i=0; i<2*count_set_bits(zyxt); i++)
    {
        send_buf[i+2] = 0x00;
    }

    res = mlx90393_transfer(dev, send_buf, recv_buf, 1, 1+2*count_set_bits(zyxt));
    if (res == RT_EOK)
    {
        int idx = 1;
        if (zyxt & 0x1)
        {
            txyz->t = ((uint16_t)recv_buf[idx]) << 8 | recv_buf[idx+1];
            idx = idx + 2;
        }

        if (zyxt & 0x2)
        {
            txyz->x = ((uint16_t)recv_buf[idx]) << 8 | recv_buf[idx+1];
            idx = idx + 2;
        }

        if (zyxt & 0x4)
        {
            txyz->y = ((uint16_t)recv_buf[idx]) << 8 | recv_buf[idx+1];
            idx = idx + 2;
        }

        if (zyxt & 0x8)
        {
            txyz->z = ((uint16_t)recv_buf[idx]) << 8 | recv_buf[idx+1];
            idx = idx + 2;
        }
    }

    return res;
}

/*
    Temperature sensor resolution       = 45.2 LSB/C
    Temperature sensor output at 25C    = 46244 LSB(16u)
    MLX90393 datasheet page 12
 */
rt_int16_t mlx90393_convert_temperature(rt_uint16_t raw)
{
    float temperature;

    temperature = 25 + (raw - 46244)/45.2;
    rt_kprintf("t = %d.%d C\r\n", (int)temperature, ((int)(temperature*10))%10);
}

rt_err_t mlx90393_convert_measurement(struct mlx90393_device *dev, struct mlx90393_txyz txyz)
{
    mlx90393_resolution_t res_x = MLX90393_RES_18;
    mlx90393_resolution_t res_y = MLX90393_RES_18;
    mlx90393_resolution_t res_z = MLX90393_RES_18;

    mlx90393_gain_t gain = 3;

    float x, y, z;

    mlx90393_get_resolution(dev, &res_x, &res_y, &res_z);

    mlx90393_get_gain_sel(dev, &gain);

    if (res_x == MLX90393_RES_18)
    {
        txyz.x -= 0x8000;
        rt_kprintf("txyz.x - 0x8000 = 0x%x\r\n", txyz.x);
    }

    if (res_x == MLX90393_RES_19)
    {
        txyz.x -= 0x4000;
        rt_kprintf("txyz.x - 0x4000 = 0x%x\r\n", txyz.x);
    }

    if (res_y == MLX90393_RES_18)
    {
        txyz.y -= 0x8000;
        rt_kprintf("txyz.y - 0x8000 = 0x%x\r\n", txyz.y);
    }

    if (res_y == MLX90393_RES_19)
    {
        txyz.y -= 0x4000;
        rt_kprintf("txyz.x - 0x4000 = 0x%x\r\n", txyz.y);
    }

    if (res_z == MLX90393_RES_18)
    {
        txyz.z -= 0x8000;
        rt_kprintf("txyz.z - 0x8000 = 0x%x\r\n", txyz.z);
    }

    if (res_z == MLX90393_RES_19)
    {
        txyz.z -= 0x4000;
        rt_kprintf("txyz.z - 0x4000 = 0x%x\r\n", txyz.z);
    }

    x = (float)txyz.x * mlx90393_lsb_lookup[0][gain][res_x][0];
    y = (float)txyz.y * mlx90393_lsb_lookup[0][gain][res_y][0];
    z = (float)txyz.z * mlx90393_lsb_lookup[0][gain][res_z][1];

    // rt_kprintf("%.3f uT %.3f uT %.3f uT\r\n", x, y, z);
    // rt_kprintf("0x%xuT 0x%xuT 0x%xuT\r\n", x, y, z);
    rt_kprintf("%duT %duT %duT\r\n", (int)x, (int)y, (int)z);
}

rt_err_t mlx90393_set_hallconf(struct mlx90393_device *dev, uint8_t hallconf)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register0 reg;

    res = mlx90393_read_reg(dev, 0, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.hallconf = hallconf;
    res = mlx90393_write_reg(dev, 0, reg.word_val);
    if (res == -RT_ERROR)
        return res;
        
    return res;
}

rt_err_t mlx90393_set_gain_sel(struct mlx90393_device *dev, mlx90393_gain_t gain)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register0 reg;

    res = mlx90393_read_reg(dev, 0, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.gain_sel = gain;
    
    res = mlx90393_write_reg(dev, 0, reg.word_val);
    if (res == -RT_ERROR)
        return res;

    return res;
}

rt_err_t mlx90393_get_gain_sel(struct mlx90393_device *dev, mlx90393_gain_t *gain)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register0 reg;

    res = mlx90393_read_reg(dev, 0, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    *gain = reg.gain_sel;
    
    rt_kprintf("gain = 0x%x\r\n", *gain);

    return res;
}

uint8_t mlx90393_set_burst_sel(struct mlx90393_device *dev, uint8_t burst_sel)
{
    uint16_t register_val;
    union mlx90393_register1 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 1, &register_val);
    reg.word_val = register_val;
    reg.burst_sel = burst_sel;
    uint8_t status2 = mlx90393_write_reg(dev, 1, reg.word_val);

    return (status1) | (status2);
}

uint8_t mlx90393_set_external_trigger(struct mlx90393_device *dev, uint8_t ext_trg)
{
    uint16_t register_val;
    union mlx90393_register1 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 1, &register_val);
    reg.word_val = register_val;
    reg.ext_trg = ext_trg;
    uint8_t status2 = mlx90393_write_reg(dev, 1, reg.word_val);

    return (status1) | (status2);
}

uint8_t mlx90393_set_trigger_interrup_sel(struct mlx90393_device *dev, uint8_t trig_int)
{
    uint16_t register_val;
    union mlx90393_register1 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 1, &register_val);
    reg.word_val = register_val;
    reg.trig_int = trig_int;
    uint8_t status2 = mlx90393_write_reg(dev, 1, reg.word_val);

    return (status1) | (status2);
}

uint8_t mlx90393_set_temperature_compensation(struct mlx90393_device *dev, uint8_t on_off)
{
    uint16_t register_val;
    union mlx90393_register1 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 1, &register_val);
    reg.word_val = register_val;
    reg.tcmp_en = on_off;
    uint8_t status2 = mlx90393_write_reg(dev, 1, reg.word_val);

    return (status1) | (status2);
}

rt_err_t mlx90393_set_resolution(struct mlx90393_device *dev, mlx90393_resolution_t res_x, mlx90393_resolution_t res_y, mlx90393_resolution_t res_z)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register2 reg;

    res = mlx90393_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.res_x = res_x;
    reg.res_y = res_y;
    reg.res_z = res_z;
    
    res = mlx90393_write_reg(dev, 2, reg.word_val);
    if (res == -RT_ERROR)
        return res;

    return res;
}

rt_err_t mlx90393_get_resolution(struct mlx90393_device *dev, mlx90393_resolution_t *res_x, mlx90393_resolution_t *res_y, mlx90393_resolution_t *res_z)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register2 reg;

    res = mlx90393_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    *res_x = reg.res_x;
    *res_y = reg.res_y;
    *res_z = reg.res_z;
    
    rt_kprintf("res_x = 0x%x, res_y = 0x%x, res_z = 0x%x\r\n", *res_x, *res_y, *res_z);

    return res;
}

rt_err_t mlx90393_set_oversampling(struct mlx90393_device *dev, mlx90393_oversampling_t osr)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register2 reg;

    res = mlx90393_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.osr = osr;
    res = mlx90393_write_reg(dev, 2, reg.word_val);
    if (res == -RT_ERROR)
        return res;

    return res;
}

rt_err_t mlx90393_get_oversampling(struct mlx90393_device *dev, mlx90393_oversampling_t *osr)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register2 reg;

    res = mlx90393_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    *osr = reg.osr;
        
    return res;
}

rt_err_t mlx90393_set_digital_filtering(struct mlx90393_device *dev, mlx90393_filter_t dig_filt)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register2 reg;

    res = mlx90393_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.dig_filt = dig_filt;
    res = mlx90393_write_reg(dev, 2, reg.word_val);
    if (res == -RT_ERROR)
        return res;

    return res;
}

rt_err_t mlx90393_get_digital_filtering(struct mlx90393_device *dev, mlx90393_filter_t *dig_filt)
{
    rt_err_t res = 0;

    uint16_t register_val;
    union mlx90393_register2 reg;

    res = mlx90393_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    *dig_filt = reg.dig_filt;

    return res;
}

uint8_t mlx90393_set_offset_x(struct mlx90393_device *dev, uint16_t offset)
{
    uint8_t status = mlx90393_write_reg(dev, 4, offset);

    return status;
}

uint8_t mlx90393_set_offset_y(struct mlx90393_device *dev, uint16_t offset)
{
    uint8_t status = mlx90393_write_reg(dev, 5, offset);

    return status;
}

uint8_t mlx90393_set_offset_z(struct mlx90393_device *dev, uint16_t offset)
{
    uint8_t status = mlx90393_write_reg(dev, 6, offset);

    return status;
}

uint8_t mlx90393_set_woxy_threshold(struct mlx90393_device *dev, uint16_t woxy_threshold)
{
    uint8_t status = mlx90393_write_reg(dev, 7, woxy_threshold);

    return status;
}

uint8_t mlx90393_set_woz_threshold(struct mlx90393_device *dev, uint16_t woz_threshold)
{
    uint8_t status = mlx90393_write_reg(dev, 6, woz_threshold);

    return status;
}

uint8_t mlx90393_set_wot_threshold(struct mlx90393_device *dev, uint16_t wot_threshold)
{
    uint8_t status = mlx90393_write_reg(dev, 6, wot_threshold);

    return status;
}

void mlx90393_setup(struct mlx90393_device *dev)
{
//    mlx90393_reset(dev);

//    rt_thread_delay(10000);

    mlx90393_set_gain_sel(dev, 4);
    mlx90393_set_resolution(dev, 0, 0, 0);
    mlx90393_set_oversampling(dev, 3);
    mlx90393_set_digital_filtering(dev, 7);
    mlx90393_set_temperature_compensation(dev, 0);
}

/**
 * This function gets the raw data of mlx90393
 *
 * @param dev the pointer of device driver structure
 * @param txyz the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
static rt_err_t mlx90393_get_txyz_raw(struct mlx90393_device *dev, struct mlx90393_txyz *txyz)
{
    uint8_t status = mlx90393_start_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);

    // wait for DRDY signal if connected, otherwise delay appropriately
//    if (DRDY_pin >= 0)
//    {
//      delayMicroseconds(600);
//      while (!digitalRead(DRDY_pin))
//      {
//        // busy wait
//      }
//    }
//    else
//    {
//      delay(this->convDelayMillis());
//    }

    status = mlx90393_read_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG, txyz);
//    data = convertRaw(raw_txyz);

    return status;
}

/**
 * This function gets mlx90393 parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param read data pointer
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
static rt_err_t mlx90393_get_param(struct mlx90393_device *dev, enum mlx90393_cmd cmd, rt_uint16_t *param)
{
    rt_uint8_t data = 0;
    rt_err_t res = RT_EOK;

    RT_ASSERT(dev);

    // switch (cmd)
    // {
    // case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, &data);
    //     *param = data;
    //     break;
    // case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, &data);
    //     *param = data;
    //     break;
    // case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
    //     *param = data;
    //     break;
    // case MPU6XXX_SAMPLE_RATE: /* Sample Rate */
    //     /* Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) */
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
    //     if (res != RT_EOK)
    //     {
    //         break;
    //     }

    //     if (data == 0 || data == 7) /* dlpf is disable */
    //     {
    //         res = mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
    //         *param = 8000 / (data + 1);
    //     }
    //     else /* dlpf is enable */
    //     {
    //         res = mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
    //         *param = 1000 / (data + 1);
    //     }
    //     break;
    // case MPU6XXX_SLEEP: /* sleep mode */
    //     res = mpu6xxx_read_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, &data);
    //     *param = data;
    //     break;
    // }

    return res;
}

/**
 * This function set mpu6xxx parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param Configuration item parameter
 *
 * @return the setting status, RT_EOK reprensents  setting the parameter successfully.
 */
rt_err_t mlx90393_set_param(struct mlx90393_device *dev, enum mlx90393_cmd cmd, rt_uint16_t param)
{
    rt_uint8_t data = 0;
    rt_err_t res = RT_EOK;

    RT_ASSERT(dev);

    // switch (cmd)
    // {
    // case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
    //     res = mpu6xxx_write_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, param);
    //     dev->config.gyro_range = param;
    //     break;
    // case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
    //     res = mpu6xxx_write_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, param);
    //     dev->config.accel_range = param;
    //     break;
    // case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
    //     res = mpu6xxx_write_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, param);
    //     break;
    // case MPU6XXX_SAMPLE_RATE: /* Sample Rate = 16-bit unsigned value.
    //                              Sample Rate = [1000 -  4]HZ when dlpf is enable
    //                              Sample Rate = [8000 - 32]HZ when dlpf is disable */

    //     //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
    //     if (res != RT_EOK)
    //     {
    //         break;
    //     }

    //     if (data == 0 || data == 7) /* dlpf is disable */
    //     {
    //         if (param > 8000)
    //             data = 0;
    //         else if (param < 32)
    //             data = 0xFF;
    //         else
    //             data = 8000 / param - 1;
    //     }
    //     else /* dlpf is enable */
    //     {
    //         if (param > 1000)
    //             data = 0;
    //         else if (param < 4)
    //             data = 0xFF;
    //         else
    //             data = 1000 / param - 1;
    //     }
    //     res = mpu6xxx_write_reg(dev, MPU6XXX_RA_SMPLRT_DIV, data);
    //     break;
    // case MPU6XXX_SLEEP: /* Configure sleep mode */
    //     res = mpu6xxx_write_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, param);
    //     break;
    // }

    return res;
}

/**
 * This function initialize the mlx90393 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct mlx90393_device *mlx90393_init(const char *dev_name, rt_uint8_t param)
{
    struct mlx90393_device *dev = RT_NULL;
    rt_uint8_t reg = 0xFF;
    rt_uint8_t res = RT_EOK;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mlx90393_device));
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't allocate memory for mlx90393 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        rt_kprintf("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            /* find mlx90393 device at address: 0x19 */
            dev->i2c_addr = MLX90393_I2C_ADDRESS;
            // if (mlx90393_read_reg(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
            // {
            //     /* find mlx90393 device at address 0x19 */
            //     dev->i2c_addr = MPU6XXX_ADDRESS_AD0_HIGH;
            //     if (mlx90393_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
            //     {
            //         rt_kprintf("Can't find device at '%s'!", dev_name);
            //         goto __exit;
            //     }
            // }
            rt_kprintf("Device i2c address is:'0x%x'!\r\n", dev->i2c_addr);
        }
#endif        
    }
    else if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
#ifdef RT_USING_SPI
        struct rt_spi_configuration cfg;

        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz = MLX90393_SPI_MAX_SPEED; /* Set spi max speed */

        rt_spi_configure((struct rt_spi_device *)dev->bus, &cfg);
#endif
    }
    else
    {
        rt_kprintf("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    if (res == RT_EOK)
    {
        rt_kprintf("Device init succeed!\r\n");
    }
    else
    {
        rt_kprintf("Error in device initialization!\r\n");
    }
    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }
    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx90393_deinit(struct mlx90393_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

static void mlx90393(int argc, char **argv)
{
    uint16_t register_val;
    static struct mlx90393_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mlx90393 [OPTION] [PARAM]\n");
        rt_kprintf("         probe <dev_name>      Probe mlx90393 by given name, ex:i2c2\n");
        rt_kprintf("         rr <reg>              Set sample rate to var\n");
        rt_kprintf("                               var = [1000 -  4] when dlpf is enable\n");
        rt_kprintf("                               var = [8000 - 32] when dlpf is disable\n");
        rt_kprintf("         wr <reg> <var>        Set gyro range to var\n");
        rt_kprintf("                               var = [0 - 3] means [250 - 2000DPS]\n");
        rt_kprintf("         ar <var>              Set accel range to var\n");
        rt_kprintf("                               var = [0 - 3] means [2 - 16G]\n");
        rt_kprintf("         sleep <var>           Set sleep status\n");
        rt_kprintf("                               var = 0 means disable, = 1 means enable\n");
        rt_kprintf("         read [num]            read [num] times mlx90393\n");
        rt_kprintf("                               num default 5\n");
        return;
    }
    else
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (dev)
            {
                mlx90393_deinit(dev);
            }

            if (argc == 2)
                dev = mlx90393_init("i2c2", RT_NULL);
            else if (argc == 3)
                dev = mlx90393_init(argv[2], RT_NULL);
        }
        else if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mlx90393 first!\n");
            return;
        }
        else if (!strcmp(argv[1], "rt"))
        {
            mlx90393_reset(dev);
        }
        else if (!strcmp(argv[1], "rr"))
        {
            union mlx90393_register0 reg0;
            union mlx90393_register1 reg1;
            union mlx90393_register2 reg2;
            union mlx90393_register3 reg3;

            rt_kprintf("Reading REG[%d]...\r\n", atoi(argv[2]));

            mlx90393_read_reg(dev, atoi(argv[2]), &register_val);

            switch (atoi(argv[2]))
            {
            case 0:
                reg0.word_val = register_val;
                rt_kprintf("REG[0] = 0x%x\r\n", reg0.word_val);
                rt_kprintf("[BIT0-3] HALLCONF = 0x%x - Hall plate spinning rate adjustment\r\n", reg0.hallconf);
                rt_kprintf("[BIT4-6] GAIN_SEL = 0x%x - Analog chain gain setting, factor 5 between min and max code\r\n", reg0.gain_sel);
                rt_kprintf("[BIT7-7] Z_SERIES = 0x%x - Enable all plates for Z-measurement\r\n", reg0.z_series);
                rt_kprintf("[BIT8-8] BITS     = 0x%x - Enable the on-chip coil, applying a Z-field[Built-in Self Test]\r\n", reg0.bist);
                rt_kprintf("[BIT9-F] ANA_RESERVED_LOW = 0x%x - Reserved IO trimming bits\r\n", reg0.ana_reserved_low);
                break;
            case 1:
                reg1.word_val = register_val;
                rt_kprintf("REG[1] = 0x%x\r\n", reg1.word_val);
                rt_kprintf("[BIT0-5] BURST_DATA_RATE = 0x%x - Defines TINTERVAL as BURST_DATA_RATE * 20ms\r\n", reg1.burst_data_rate);
                rt_kprintf("[BIT6-9] BURST_SEL       = 0x%x - Defines the MDATA in burst mode if SB command argument = 0\r\n", reg1.burst_sel);
                rt_kprintf("[BITA-A] TCMP_EN         = 0x%x - Enables on-chip sensitivity drift compensation\r\n", reg1.tcmp_en);
                rt_kprintf("[BITB-B] EXT_TRG         = 0x%x - Allows external trigger inputs when set, if TRIG_INT_SEL = 0\r\n", reg1.ext_trg);
                rt_kprintf("[BITC-C] WOC_DIFF        = 0x%x - Sets the Wake-up On Change based on Δ{sample(t),sample(t-1)}\r\n", reg1.woc_diff);
                rt_kprintf("[BITD-E] COMM_MODE       = 0x%x - Allow only SPI [10b], only I2C [11b] or both [0Xb] according to CS pin\r\n", reg1.comm_mode);
                rt_kprintf("[BITF-F] TRIG_INT        = 0x%x - Puts TRIG_INT pin in TRIG mode when cleared, INT mode otherwise\r\n", reg1.trig_int);
                break;
            case 2:
                reg2.word_val = register_val;
                rt_kprintf("REG[2] = 0x%x\r\n", reg2.word_val);
                rt_kprintf("[BIT0-1] OSR      = 0x%x - Magnetic sensor ADC oversampling ratio\r\n", reg2.osr);
                rt_kprintf("[BIT2-4] DIG_FILT = 0x%x - Digital filter applicable to ADC\r\n", reg2.dig_filt);
                rt_kprintf("[BIT5-6] RES_X    = 0x%x - Selects the desired 16-bit output value from the 19-bit ADC\r\n", reg2.res_x);
                rt_kprintf("[BIT7-8] RES_Y    = 0x%x - Selects the desired 16-bit output value from the 19-bit ADC\r\n", reg2.res_y);
                rt_kprintf("[BIT9-A] RES_Z    = 0x%x - Selects the desired 16-bit output value from the 19-bit ADC\r\n", reg2.res_z);
                rt_kprintf("[BITB-C] OSR_2    = 0x%x - Temperature sensor ADC oversampling ratio\r\n", reg2.osr2);
                break;
            case 3:
                reg3.word_val = register_val;
                rt_kprintf("REG[3] = 0x%x\r\n", reg3.word_val);
                rt_kprintf("[BIT0-7] SENS_TC_LT = 0x%x - Sensitivity drift compensation factor for T > TREF\r\n", reg3.sens_tc_lt);
                rt_kprintf("[BIT8-F] SENS_TC_HT = 0x%x - Sensitivity drift compensation factor for T < TREF\r\n", reg3.sens_tc_ht);
                break;
            default:
                rt_kprintf("REG[%d] = 0x%x\r\n", atoi(argv[2]), register_val);
                break;
            }
        }
        else if (!strcmp(argv[1], "wr"))
        {
            mlx90393_write_reg(dev, atoi(argv[2]), atoi(argv[3]));
        }
        else if (!strcmp(argv[1], "sm"))
        {
            mlx90393_start_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);
        }        
        else if (!strcmp(argv[1], "rm"))
        {
            struct mlx90393_txyz txyz;

            mlx90393_start_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);
            
            // rt_thread_delay(mlx90393_tconv[_dig_filt][_osr] + 10);
            rt_thread_delay(mlx90393_tconv[0][0] + 10);

            mlx90393_read_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG, &txyz);

            mlx90393_convert_temperature(txyz.t);
            mlx90393_convert_measurement(dev, txyz);
        }                
        else if (!strcmp(argv[1], "set_gain"))
        {
            mlx90393_set_gain_sel(dev, atoi(argv[2]));
        }                
        else if (!strcmp(argv[1], "get_gain"))
        {
            mlx90393_gain_t gain;

            mlx90393_get_gain_sel(dev, &gain);
            rt_kprintf("gain is 0x%x\r\n", gain);
        }                                
        else if (!strcmp(argv[1], "set_resolution"))
        {
            mlx90393_set_resolution(dev, atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        }                
        else if (!strcmp(argv[1], "get_resolution"))
        {
            mlx90393_resolution_t x;
            mlx90393_resolution_t y;
            mlx90393_resolution_t z;

            mlx90393_get_resolution(dev, &x, &y, &z);
            rt_kprintf("resolution is 0x%x 0x%x 0x%x\r\n", x, y, z);
        }                                        
        else if (!strcmp(argv[1], "setup"))
        {
            mlx90393_setup(dev);
        }
        else if (!strcmp(argv[1], "readdata"))
        {
            struct mlx90393_txyz txyz;
            mlx90393_get_txyz_raw(dev, &txyz);
        }
        else
        {
            rt_kprintf("Unknown command, please enter 'mlx90393' get help information!\n");
        }
    }
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(mlx90393, mlx90393 sensor function);    
#endif

#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx90393, mlx90393 sensor function);
#endif
