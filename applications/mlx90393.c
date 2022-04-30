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

#include <string.h>
#include <stdlib.h>

#define DBG_TAG "mlx90393"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "mlx90393.h"

rt_err_t mlx90393_i2c_cmd(struct mlx90393_device *dev, enum cmd c)
{
    rt_int8_t res = 0;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];

        write_buffer[0] = c;

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif

    return res;
}

rt_err_t mlx90393_nop(struct mlx90393_device *dev)
{
    rt_int8_t res = 0;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = CMD_NOP;

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

rt_err_t mlx90393_exit(struct mlx90393_device *dev)
{
    rt_int8_t res = 0;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = CMD_EXIT;

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

rt_err_t mlx90393_start_burst(struct mlx90393_device *dev, rt_int8_t zyxt)
{
    rt_int8_t res = 0;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = (CMD_START_BURST)|(zyxt);

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

rt_err_t mlx90393_wake_on_change(struct mlx90393_device *dev, rt_int8_t zyxt)
{
    rt_int8_t res = 0;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = (CMD_WAKE_ON_CHANGE)|(zyxt);

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

rt_err_t mlx90393_start_measurement(struct mlx90393_device *dev, rt_int8_t zyxt)
{
    rt_int8_t res = 0;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = (CMD_START_MEASUREMENT)|(zyxt);

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

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
    rt_int8_t res = 0;
    union mlx90393_status status;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = (CMD_READ_MEASUREMENT)|(zyxt);

        for (int i=0; i<2*count_set_bits(zyxt); i++)
        {
            write_buffer[i+2] = 0x00;
        }

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1+2*count_set_bits(zyxt);                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            status.byte_val = read_buffer[0];
            rt_kprintf("status = 0x%x, BIT4(ERROR) = %d, D1D0 = %d\r\n", status.byte_val, status.error, status.d1<<2|status.d0);

            int idx = 1;
            if (zyxt & 0x1)
            {
                txyz->t = ((uint16_t)read_buffer[idx]) << 8 | read_buffer[idx+1];
                idx = idx + 2;
            }

            if (zyxt & 0x2)
            {
                txyz->x = ((uint16_t)read_buffer[idx]) << 8 | read_buffer[idx+1];
                idx = idx + 2;
            }

            if (zyxt & 0x4)
            {
                txyz->y = ((uint16_t)read_buffer[idx]) << 8 | read_buffer[idx+1];
                idx = idx + 2;
            }

            if (zyxt & 0x8)
            {
                txyz->z = ((uint16_t)read_buffer[idx]) << 8 | read_buffer[idx+1];
                idx = idx + 2;
            }

            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

/**
 * This function reads the value of register for mlx90393
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90393
 * @param buf read data pointer
 *
 * @return the reading status, RT_EOK represents  reading the value of register successfully.
 */
static rt_err_t mlx90393_read_reg(struct mlx90393_device *dev, rt_uint8_t reg, rt_uint16_t *val)
{
    rt_int8_t res = 0;
    union mlx90393_status status;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[3];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = CMD_READ_REGISTER;
        write_buffer[1] = reg << 2;

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR | RT_I2C_NO_STOP;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 2;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 3;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            status.byte_val = read_buffer[0];
            rt_kprintf("status = 0x%x\r\n", status.byte_val);
            rt_kprintf("[BIT7] BURST_MODE = 0x%x - MLX90393 works in Burst mode\r\n", status.burst_mode);
            rt_kprintf("[BIT6] WOC_MODE   = 0x%x - MLX90393 works in Wake On Change mode\r\n", status.woc_mode);
            rt_kprintf("[BIT5] SM_MODE    = 0x%x - MLX90393 works in Single measurement mode\r\n", status.sm_mode);
            rt_kprintf("[BIT4] ERROR      = 0x%x - ECC_ERROR or command is rejected\r\n", status.error);
            rt_kprintf("[BIT3] SED        = 0x%x - a bit error in the non-volatile memory has been corrected\r\n", status.sed);
            rt_kprintf("[BIT2] RS         = 0x%x - Reset bit\r\n", status.rs);
            rt_kprintf("[BIT1] D1         = 0x%x - The number of response bytes correspond to 2*D[1:0]+2\r\n", status.d1);
            rt_kprintf("[BIT0] D0         = 0x%x - The number of response bytes correspond to 2*D[1:0]+2\r\n\r\n", status.d0);

            *val = ((uint16_t)read_buffer[1])<<8 | read_buffer[2];
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

/**
 * This function writes the value of the register for mlx90393
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90393
 * @param data value to write
 *
 * @return the writing status, RT_EOK represents writing the value of the register successfully.
 */
static rt_err_t mlx90393_write_reg(struct mlx90393_device *dev, rt_uint8_t reg, rt_uint16_t data)
{
    rt_int8_t res = 0;
    union mlx90393_status status;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
    rt_uint8_t read_buffer;
    rt_uint8_t write_buffer[4] =
    {
        CMD_WRITE_REGISTER,
        (data&0xFF00) >> 8,
        data&0x00FF,
        reg << 2
    };
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs[0].addr  = dev->i2c_addr;    /* slave address */
        msgs[0].flags = RT_I2C_WR;        /* write flag */
        msgs[0].buf   = write_buffer;     /* Send data pointer */
        msgs[0].len   = 4;

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = &read_buffer;     /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            status.byte_val = read_buffer;
            rt_kprintf("status = 0x%x, BIT4(ERROR) = %d\r\n", status.byte_val, status.error);

            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        res = rt_spi_send_then_send((struct rt_spi_device *)dev->bus, &reg, 1, &data, 1);
#endif
    }

    return res;
}

rt_err_t mlx90393_memory_recall(struct mlx90393_device *dev)
{
    rt_int8_t res = 0;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = CMD_MEMORY_RECALL;

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

rt_err_t mlx90393_memory_store(struct mlx90393_device *dev)
{
    rt_int8_t res = 0;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = CMD_MEMORY_STORE;

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

rt_err_t mlx90393_reset(struct mlx90393_device *dev)
{
    rt_int8_t res = 0;
    union mlx90393_status status;

#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];

    uint8_t write_buffer[10];
    uint8_t read_buffer[10];
#endif

#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        write_buffer[0] = CMD_RESET;

        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = write_buffer;     /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;      /* Read data pointer */
        msgs[1].len   = 1;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            status.byte_val = read_buffer[0];
            rt_kprintf("status = 0x%x, BIT2(RS) = %d, BIT4(ERROR) = %d\r\n", status.byte_val, status.rs, status.error);

            //if (buf[0] == 0x00)
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf);
#endif
    }

    return res;
}

uint8_t mlx90393_set_hallconf(struct mlx90393_device *dev, uint8_t hallconf)
{
    uint16_t register_val;
    union mlx90393_register0 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 0, &register_val);
    reg.word_val = register_val;
    reg.hallconf = hallconf;
    uint8_t status2 = mlx90393_write_reg(dev, 0, reg.word_val);

    return (status1) | (status2);
}

uint8_t mlx90393_set_gain_sel(struct mlx90393_device *dev, uint8_t gain)
{
    uint16_t register_val;
    union mlx90393_register0 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 0, &register_val);
    reg.word_val = register_val;
    rt_kprintf("reg0 = 0x%x\r\n", reg.word_val);
    reg.gain_sel = gain;
    rt_kprintf("reg0 = 0x%x\r\n", reg.word_val);
    uint8_t status2 = mlx90393_write_reg(dev, 0, reg.word_val);

    return (status1) | (status2);
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

uint8_t mlx90393_set_resolution(struct mlx90393_device *dev, uint8_t res_x, uint8_t res_y, uint8_t res_z)
{
    uint16_t register_val;
    union mlx90393_register2 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 2, &register_val);
    reg.word_val = register_val;
    reg.res_x = res_x;
    reg.res_y = res_y;
    reg.res_z = res_z;
    uint8_t status2 = mlx90393_write_reg(dev, 2, reg.word_val);

    return (status1) | (status2);
}

uint8_t mlx90393_set_oversampling(struct mlx90393_device *dev, uint8_t osr)
{
    uint16_t register_val;
    union mlx90393_register2 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 2, &register_val);
    reg.word_val = register_val;
    reg.osr = osr;
    uint8_t status2 = mlx90393_write_reg(dev, 2, reg.word_val);

    return (status1) | (status2);
}

uint8_t mlx90393_set_digital_filtering(struct mlx90393_device *dev, uint8_t dig_filt)
{
    uint16_t register_val;
    union mlx90393_register2 reg;

    uint8_t status1 = mlx90393_read_reg(dev, 2, &register_val);
    reg.word_val = register_val;
    reg.dig_filt = dig_filt;
    uint8_t status2 = mlx90393_write_reg(dev, 2, reg.word_val);

    return (status1) | (status2);
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
 * This function gets the data of the gyroscope, unit: deg/10s
 * Here deg/10s means 10 times higher precision than deg/s.
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
//rt_err_t mlx90393_get_gyro(struct mlx90393_device *dev, struct mlx90393_3axes *gyro)
//{
//    struct mlx90393_3axes tmp;
//    rt_uint16_t sen;
//    rt_err_t res;
//
//    res = mlx90393_get_gyro_raw(dev, &tmp);
//    if (res != RT_EOK)
//    {
//        return res;
//    }
//
////    sen = MPU6XXX_GYRO_SEN >> dev->config.gyro_range;
////
////    gyro->x = (rt_int32_t)tmp.x * 100 / sen;
////    gyro->y = (rt_int32_t)tmp.y * 100 / sen;
////    gyro->z = (rt_int32_t)tmp.z * 100 / sen;
//
//    return RT_EOK;
//}

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
        LOG_E("Can't allocate memory for mlx90393 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
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
            //         LOG_E("Can't find device at '%s'!", dev_name);
            //         goto __exit;
            //     }
            // }
            LOG_D("Device i2c address is:'0x%x'!", dev->i2c_addr);
            rt_kprintf("Device i2c address is:'0x%x'!\r\n", dev->i2c_addr);
        }
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
        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    // if (mlx90393_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
    // {
    //     LOG_E("Failed to read device id!");
    //     goto __exit;
    // }

    // dev->id = reg;

    // switch (dev->id)
    // {
    // case MPU6050_WHO_AM_I:
    //     LOG_I("Find device: mpu6050!");
    //     break;
    // case 0xFF:
    //     LOG_E("No device connection!");
    //     goto __exit;
    // default:
    //     LOG_W("Unknown device id: 0x%x!", reg);
    // }

    // res += mpu6xxx_get_param(dev, MPU6XXX_ACCEL_RANGE, &dev->config.accel_range);
    // res += mpu6xxx_get_param(dev, MPU6XXX_GYRO_RANGE, &dev->config.gyro_range);

    // res += mpu6xxx_write_bits(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_CLKSEL_BIT, MPU6XXX_PWR1_CLKSEL_LENGTH, MPU6XXX_CLOCK_PLL_XGYRO);
    // res += mpu6xxx_set_param(dev, MPU6XXX_GYRO_RANGE, MPU6XXX_GYRO_RANGE_250DPS);
    // res += mpu6xxx_set_param(dev, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_2G);
    // res += mpu6xxx_set_param(dev, MPU6XXX_SLEEP, MPU6XXX_SLEEP_DISABLE);

    if (res == RT_EOK)
    {
        LOG_I("Device init succeed!");
    }
    else
    {
        LOG_W("Error in device initialization!");
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
        return ;
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
            return ;
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
                rt_kprintf("REG[1] = 0x%x, BURST_DATA_RATE = 0x%x, BURST_SEL = 0x%x, TCMP_EN = 0x%x, EXT_TRG = 0x%x, WOC_DIFF = 0x%x, COMM_MODE = 0x%x, TRIG_INT = 0x%x\r\n", reg1.word_val, reg1.burst_data_rate, reg1.burst_sel, reg1.tcmp_en, reg1.ext_trg, reg1.woc_diff, reg1.comm_mode, reg1.trig_int);
                break;
            case 2:
                reg2.word_val = register_val;
                rt_kprintf("REG[2] = 0x%x, OSR = 0x%x, DIG_FILT = 0x%x, RES_X = 0x%x, RES_Y = 0x%x, RES_Z = 0x%x, OSR2 = 0x%x\r\n", reg2.word_val, reg2.osr, reg2.dig_filt, reg2.res_x, reg2.res_y, reg2.res_z, reg2.osr2);
                break;
            case 3:
                reg3.word_val = register_val;
                rt_kprintf("REG[3] = 0x%x, SENS_TC_LT = 0x%x, SENS_TC_HT = 0x%x\r\n", reg3.word_val, reg3.sens_tc_lt, reg3.sens_tc_ht);
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
