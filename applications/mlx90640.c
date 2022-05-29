/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-25     lgnq         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "mlx90640.h"

#include <string.h>
#include <stdlib.h>

rt_err_t mlx90640_reset(struct mlx90640_device *dev)
{
    rt_err_t res = RT_EOK;
    struct rt_i2c_msg msg;

    rt_uint8_t send_buf[10];

    send_buf[0] = 0x06;

    msg.addr  = 0x0;              /* I2C Slave address */
    msg.flags = RT_I2C_WR;        /* Write flag */
    msg.buf   = send_buf;         /* Write data pointer */
    msg.len   = 1;                /* Number of bytes write */

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msg, 1) == 1)
    {
        rt_kprintf("success\r\n");

        res = RT_EOK;
    }
    else
    {
        rt_kprintf("failed\r\n");
        
        res = RT_ERROR;
    }

    return res;
}

/**
 * This function reads the value of register for mlx90640
 *
 * @param dev  the pointer of device driver structure
 * @param addr the start address
 * @param data the data buffer
 * @param num  the number for reading
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
static rt_err_t mlx90640_read(struct mlx90640_device *dev, rt_uint16_t addr, rt_uint16_t *data, rt_uint16_t num)
{
    rt_err_t res = RT_EOK;
    struct rt_i2c_msg msgs[2];

    rt_uint8_t send_buf[10];
    rt_uint8_t *recv_buf = (rt_uint8_t *)data;

    send_buf[0] = addr >> 8;
    send_buf[1] = addr & 0xff;

    msgs[0].addr  = dev->i2c_addr;    /* I2C Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = send_buf;         /* Write data pointer */
    msgs[0].len   = 2;                /* Number of bytes write */

    msgs[1].addr  = dev->i2c_addr;    /* I2C Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Write flag */
    msgs[1].buf   = recv_buf;         /* Write data pointer */
    msgs[1].len   = num*2;            /* Number of bytes write */
    
    if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
    {
        for (rt_uint16_t i=0; i < num*2;)
        {
            *data++ = (rt_uint16_t)recv_buf[i]*256 + (rt_uint16_t)recv_buf[i+1];
            i = i + 2;
        }

        res = RT_EOK;
    }
    else
    {
        rt_kprintf("failed\r\n");
        
        res = RT_ERROR;
    }

    return res;
}

/**
 * This function writes the value of the register for mlx90640
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90640
 * @param val value to write
 *
 * @return the writing status, RT_EOK represents writing the value of the register successfully.
 */
static rt_err_t mlx90640_write(struct mlx90640_device *dev, rt_uint16_t addr, rt_uint16_t data)
{
    rt_err_t res = RT_EOK;
    struct rt_i2c_msg msg;

    rt_uint8_t send_buf[] =
    {
        addr >> 8,
        addr & 0xff,
        data >> 8,
        data & 0xff
    };

    msg.addr  = dev->i2c_addr;    /* I2C Slave address */
    msg.flags = RT_I2C_WR;        /* Write flag */
    msg.buf   = send_buf;         /* Write data pointer */
    msg.len   = 4;                /* Number of bytes write */

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msg, 1) == 1)
    {
        rt_kprintf("success\r\n");

        res = RT_EOK;
    }
    else
    {
        rt_kprintf("failed\r\n");
        
        res = RT_ERROR;
    }

    return res;
}

void mlx90640_read_id(struct mlx90640_device *dev)
{
    rt_uint16_t id[3];

    mlx90640_read(dev, 0x2407, id, 3);

    rt_kprintf("Read MLX90640 ID ... \r\n");

    for (int i=0; i<3; i++)
    {
        rt_kprintf("Device ID%d = 0x%04x\r\n", i, id[i]);
    }    
}

rt_err_t mlx90640_dump_eeprom(struct mlx90640_device *dev, rt_uint16_t *eeprom)
{
    return mlx90640_read(dev, EEPROM_START_ADDR, eeprom, EEPROM_LENGTH);
}

rt_err_t mlx90640_get_current_resolution(struct mlx90640_device *dev)
{
    union mlx90640_control_register1 reg;
    rt_uint16_t val;
    rt_err_t res = RT_EOK;
    
    res = mlx90640_read(dev, CONTROL_REGISTER_1_ADDR, &val, 1);
    if (res == RT_EOK)
    {
        reg.word_val = val;
        dev->resolution = reg.resolution;

#ifdef MLX90640_DEBUG
        switch (dev->resolution)
        {
            case ADC_SET_TO_16_BIT_RESOLUTION:
                rt_kprintf("Resolution is 16 bits\r\n");
                break;
            case ADC_SET_TO_17_BIT_RESOLUTION:
                rt_kprintf("Resolution is 17 bits\r\n");
                break;
            case ADC_SET_TO_18_BIT_RESOLUTION:
                rt_kprintf("Resolution is 18 bits\r\n");
                break;
            case ADC_SET_TO_19_BIT_RESOLUTION:
                rt_kprintf("Resolution is 19 bits\r\n");
                break;                
            default:
                rt_kprintf("Resolution is wrong\r\n");
                break;                                                                                   
        }
#endif
    }

    return res; 
}

rt_err_t mlx90640_set_current_resolution(struct mlx90640_device *dev, enum mlx90640_resolution resolution)
{
    union mlx90640_control_register1 reg;
    rt_uint16_t val;
    rt_err_t res = RT_EOK;
    
    res = mlx90640_read(dev, CONTROL_REGISTER_1_ADDR, &val, 1);
    if (res == RT_EOK)
    {
        reg.word_val = val;
        reg.resolution = resolution;

        res = mlx90640_write(dev, CONTROL_REGISTER_1_ADDR, reg.word_val);        
    }
    
    return res;
}

rt_err_t mlx90640_get_refresh_rate(struct mlx90640_device *dev)
{
    union mlx90640_control_register1 reg;
    rt_uint16_t val;
    rt_err_t res = RT_EOK;
    
    res = mlx90640_read(dev, CONTROL_REGISTER_1_ADDR, &val, 1);
    if (res == RT_EOK)
    {
        reg.word_val = val;
        dev->refresh_rate = reg.refresh_rate;

#ifdef MLX90640_DEBUG
        switch (dev->refresh_rate)
        {
            case IR_REFRESH_RATE_0_5_HZ:
                rt_kprintf("Refresh rate is 0.5 Hz\r\n");
                break;
            case IR_REFRESH_RATE_1_HZ:
                rt_kprintf("Refresh rate is 1 Hz\r\n");
                break;
            case IR_REFRESH_RATE_2_HZ:
                rt_kprintf("Refresh rate is 2 Hz\r\n");
                break;
            case IR_REFRESH_RATE_4_HZ:
                rt_kprintf("Refresh rate is 4 Hz\r\n");
                break;
            case IR_REFRESH_RATE_8_HZ:
                rt_kprintf("Refresh rate is 8 Hz\r\n");
                break;
            case IR_REFRESH_RATE_16_HZ:
                rt_kprintf("Refresh rate is 16 Hz\r\n");
                break;
            case IR_REFRESH_RATE_32_HZ:
                rt_kprintf("Refresh rate is 32 Hz\r\n");
                break;
            case IR_REFRESH_RATE_64_HZ:
                rt_kprintf("Refresh rate is 64 Hz\r\n");
                break;                         
            default:
                rt_kprintf("Refresh rate is wrong\r\n");
                break;                                                                                   
        }
#endif
    }
    
    return res;
}

rt_err_t mlx90640_set_refresh_rate(struct mlx90640_device *dev, enum mlx90640_refresh_rate refresh_rate)
{
    union mlx90640_control_register1 reg;
    rt_uint16_t val;
    rt_err_t res = RT_EOK;
    
    res = mlx90640_read(dev, CONTROL_REGISTER_1_ADDR, &val, 1);
    if (res == RT_EOK)
    {
        reg.word_val = val;
        reg.refresh_rate = refresh_rate;
        res = mlx90640_write(dev, CONTROL_REGISTER_1_ADDR, reg.word_val);
    }    
    
    return res;
}

rt_err_t mlx90640_get_reading_pattern(struct mlx90640_device *dev)
{
    union mlx90640_control_register1 reg;
    rt_uint16_t val;
    rt_err_t res = RT_EOK;
    
    res = mlx90640_read(dev, CONTROL_REGISTER_1_ADDR, &val, 1);
    if (res == RT_EOK)
    {
        reg.word_val = val;
        dev->reading_pattern = reg.reading_pattern;
        rt_kprintf("current reading pattern is 0x%x\r\n", dev->reading_pattern);
    }    
    
    return res;
}

rt_err_t mlx90640_set_reading_pattern(struct mlx90640_device *dev, enum mlx90640_reading_pattern reading_pattern)
{
    union mlx90640_control_register1 reg;
    rt_uint16_t val;
    rt_err_t res = RT_EOK;
    
    res = mlx90640_read(dev, CONTROL_REGISTER_1_ADDR, &val, 1);
    if (res == RT_EOK)
    {
        reg.word_val = val;
        reg.reading_pattern = reading_pattern;
        res = mlx90640_write(dev, CONTROL_REGISTER_1_ADDR, reg.word_val);
    }    
    
    return res;
}

rt_err_t mlx90640_get_vdd_param(struct mlx90640_device *dev)
{
    rt_err_t res = RT_EOK;
    rt_uint16_t val;
    
    res = mlx90640_read(dev, 0x2451, &val, 1);
    if (res == RT_EOK)
    {
        dev->kvdd = (val&0xFF00)>>8;
        if (dev->kvdd > 127)
        {
            dev->kvdd = dev->kvdd -256;
        }

        dev->kvdd = 32 * dev->kvdd;

        dev->vdd25 = val & 0xFF;
        dev->vdd25 = ((dev->vdd25 - 256) << 5) - 8192;

#ifdef MLX90640_DEBUG
        rt_kprintf("kVdd: 0x%x\r\n", dev->kvdd);
        rt_kprintf("vdd25: 0x%x\r\n", dev->vdd25);
#endif        
    }

    return res;
}

rt_err_t mlx90640_setup(struct mlx90640_device *dev)
{
    rt_err_t res = RT_EOK;

    rt_uint16_t eeprom[EEPROM_LENGTH];

    mlx90640_reset(dev);
    mlx90640_read_id(dev);

    res = mlx90640_dump_eeprom(dev, eeprom);
    if (res != RT_EOK)
    {
        return res;
    }

#ifdef MLX90640_DEBUG
    for (int i=0; i<832; i++)
    {
        rt_kprintf("0x%04x, ", eeprom[i]);

        if (i%16 == 15)
            rt_kprintf("\r\n");
    }
#endif

    mlx90640_get_vdd_param(dev);

    mlx90640_set_current_resolution(dev, ADC_SET_TO_19_BIT_RESOLUTION);
    mlx90640_get_current_resolution(dev);
    mlx90640_set_refresh_rate(dev, IR_REFRESH_RATE_64_HZ);
    mlx90640_get_refresh_rate(dev);
    mlx90640_get_reading_pattern(dev);

    return res;
}

/**
 * This function initialize the mlx90640 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication
 *
 * @return the pointer of device driver structure, RT_NULL represents initialization failed.
 */
struct mlx90640_device *mlx90640_init(const char *dev_name, rt_uint8_t param)
{
    struct mlx90640_device *dev = RT_NULL;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mlx90640_device));
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't allocate memory for mlx90640 device on '%s' ", dev_name);
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
            /* find mlx90640 device at address: 0x33 */
            dev->i2c_addr = MLX90640_I2C_ADDRESS;

            mlx90640_setup(dev);

            // if (mlx90640_read_reg(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
            // {
            //     /* find mlx90640 device at address 0x19 */
            //     dev->i2c_addr = MPU6XXX_ADDRESS_AD0_HIGH;
            //     if (mlx90640_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
            //     {
            //         rt_kprintf("Can't find device at '%s'!", dev_name);
            //         goto __exit;
            //     }
            // }
            rt_kprintf("Device i2c address is:'0x%x'!\r\n", dev->i2c_addr);
        }
#endif   
    }     
    else
    {
        rt_kprintf("Unsupported device:'%s'!", dev_name);
        goto __exit;
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
void mlx90640_deinit(struct mlx90640_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

static void mlx90640(int argc, char **argv)
{
    rt_uint16_t register_val;
    static struct mlx90640_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mlx90640 [OPTION] [PARAM]\n");
        rt_kprintf("         probe <dev_name>      Probe mlx90640 by given name, ex:i2c2\n");
        rt_kprintf("         rr <reg>              Set sample rate to var\n");
        rt_kprintf("                               var = [1000 -  4] when dlpf is enable\n");
        rt_kprintf("                               var = [8000 - 32] when dlpf is disable\n");
        rt_kprintf("         wr <reg> <var>        Set gyro range to var\n");
        rt_kprintf("                               var = [0 - 3] means [250 - 2000DPS]\n");
        rt_kprintf("         ar <var>              Set accel range to var\n");
        rt_kprintf("                               var = [0 - 3] means [2 - 16G]\n");
        rt_kprintf("         sleep <var>           Set sleep status\n");
        rt_kprintf("                               var = 0 means disable, = 1 means enable\n");
        rt_kprintf("         read [num]            read [num] times mlx90640\n");
        rt_kprintf("                               num default 5\n");
        return;
    }
    else
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (dev)
            {
                mlx90640_deinit(dev);
            }

            if (argc == 2)
                dev = mlx90640_init("i2c1", RT_NULL);
            else if (argc == 3)
                dev = mlx90640_init(argv[2], RT_NULL);
        }
        else if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mlx90640 first!\n");
            return;
        }
        else if (!strcmp(argv[1], "rt"))
        {
            mlx90640_reset(dev);
        }                                        
        else if (!strcmp(argv[1], "setup"))
        {
            mlx90640_setup(dev);
        }
        else
        {
            rt_kprintf("Unknown command, please enter 'mlx90640' get help information!\n");
        }
    }
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(mlx90640, mlx90640 sensor function);    
#endif

#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx90640, mlx90640 sensor function);
#endif
