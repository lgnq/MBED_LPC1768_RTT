/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-11     lgnq         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"

#ifdef RT_USING_I2C

#ifdef RT_USING_I2C_BITOPS

struct lpc_i2c_bit_data
{
    struct
    {
        uint32_t port;
        uint32_t pin;
    } scl, sda;
};

static void gpio_set_sda(void *data, rt_int32_t state)
{
    struct lpc_i2c_bit_data *bd = data;

    if (state)
    {
        //bd->sda.base->B[bd->sda.port][bd->sda.pin] = 1;
        GPIO_SetValue(bd->sda.port, 1<<bd->sda.pin);
    }
    else
    {
        GPIO_ClearValue(bd->sda.port, 1<<bd->sda.pin);
    }
}

static void gpio_set_scl(void *data, rt_int32_t state)
{
    struct lpc_i2c_bit_data *bd = data;

    if (state)
    {
        //bd->scl.base->B[bd->sda.port][bd->sda.pin] = 1;
        GPIO_SetValue(bd->scl.port, 1<<bd->scl.pin);
    }
    else
    {
        //bd->scl.base->B[bd->sda.port][bd->sda.pin] = 0;
        GPIO_ClearValue(bd->sda.port, 1<<bd->scl.pin);
    }
}

static rt_int32_t gpio_get_sda(void *data)
{
    struct lpc_i2c_bit_data *bd = data;

    return GPIO_ReadValue(bd->sda.port) & (0x01<<bd->sda.pin);
}

static rt_int32_t gpio_get_scl(void *data)
{
    struct lpc_i2c_bit_data *bd = data;

    return GPIO_ReadValue(bd->scl.port) & (0x01<<bd->scl.pin);
}

static void gpio_udelay(rt_uint32_t us)
{
    volatile rt_int32_t i;

    for (; us > 0; us--)
    {
        i = 10;
        while (i--);
    }
}

int rt_hw_i2c_init(void)
{
    /* register I2C0: SCL/P0_28 SDA/P0_27 */
    /* register I2C1: SCL/P0_01 SDA/P0_00 */
    /* register I2C2: SCL/P0_11 SDA/P0_10 */
    {
        static struct rt_i2c_bus_device i2c_device;

        static const struct lpc_i2c_bit_data _i2c_bdata =
        {
            /* SCL2 */ {0, 11},
            /* SDA2 */ {0, 10},
        };

        static const struct rt_i2c_bit_ops _i2c_bit_ops =
        {
            (void*)&_i2c_bdata,
            gpio_set_sda,
            gpio_set_scl,
            gpio_get_sda,
            gpio_get_scl,

            gpio_udelay,

            5,
            100
        };

        GPIO_SetDir(_i2c_bdata.sda.port, (1<<_i2c_bdata.sda.pin), 1);
        GPIO_SetDir(_i2c_bdata.scl.port, (1<<_i2c_bdata.scl.pin), 1);

        PINSEL_CFG_Type PinCfg;

        /*
        * Init I2C pin connect
        */
        PinCfg.OpenDrain = 1;
        PinCfg.Pinmode = 0;

        PinCfg.Funcnum = 0;
        PinCfg.Pinnum = _i2c_bdata.sda.pin;
        PinCfg.Portnum = _i2c_bdata.sda.port;
        PINSEL_ConfigPin(&PinCfg);
        PinCfg.Pinnum = _i2c_bdata.scl.pin;
        PINSEL_ConfigPin(&PinCfg);        

        i2c_device.priv = (void *)&_i2c_bit_ops;
        rt_i2c_bit_add_bus(&i2c_device, "i2c2");

        rt_kprintf("software simulation %s init done, pin scl: %d, pin sda %d", "i2c2", _i2c_bdata.scl.pin, _i2c_bdata.sda.pin);
    } /* register I2C */

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#else /* RT_USING_I2C_BITOPS */

struct lpc_i2c_bus
{
    struct rt_i2c_bus_device parent;
    LPC_I2C_TypeDef *I2C;
};

static rt_size_t lpc_i2c_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    I2C_M_SETUP_Type xfer = {0};
    rt_uint32_t i;
    rt_err_t ret = RT_ERROR;

    struct lpc_i2c_bus *lpc_i2c = (struct lpc_i2c_bus *)bus;

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags & RT_I2C_RD)
        {
            xfer.sl_addr7bit = msg->addr;
            xfer.rx_data     = msg->buf;
            xfer.rx_length   = msg->len;
            xfer.retransmissions_max = 3;
            if (I2C_MasterTransferData(lpc_i2c->I2C, &xfer, I2C_TRANSFER_POLLING) != SUCCESS)
            {
                rt_kprintf("i2c bus read failed, i2c bus stop!\n");
                goto out;
            }
        }
        else
        {
            xfer.sl_addr7bit = msg->addr;
            xfer.tx_data     = msg->buf;
            xfer.tx_length   = msg->len;
            xfer.retransmissions_max = 3;

            // if (!(msg->flags & RT_I2C_NO_STOP))
            {
                if (I2C_MasterTransferData(lpc_i2c->I2C, &xfer, I2C_TRANSFER_POLLING) != SUCCESS)
                {
                    rt_kprintf("i2c bus write failed, i2c bus stop!\n");
                    goto out;
                }
            }
        }
    }
    ret = i;

out:
    // i2c_dbg("send stop condition\n");

    return ret;
}

static const struct rt_i2c_bus_device_ops i2c_ops =
{
    lpc_i2c_xfer,
    RT_NULL,
    RT_NULL
};

int rt_hw_i2c_init(void)
{
    static struct lpc_i2c_bus lpc_i2c2;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Init I2C pin connect
	 */
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;

    /* register I2C0: SCL/P0_28 SDA/P0_27 */
    /* register I2C1: SCL/P0_01 SDA/P0_00 */
    /* register I2C2: SCL/P0_11 SDA/P0_10 */

#ifdef BSP_USING_I2C0
	PinCfg.Funcnum = 1;
	PinCfg.Pinnum = 27;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 28;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize Slave I2C peripheral
	I2C_Init(LPC_I2C0, 100000);

	/* Enable Slave I2C operation */
	I2C_Cmd(LPC_I2C0, ENABLE);    
#endif

#ifdef BSP_USING_I2C1
	PinCfg.Funcnum = 3;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

    // Initialize Slave I2C peripheral
	I2C_Init(LPC_I2C1, 100000);

	/* Enable Slave I2C operation */
	I2C_Cmd(LPC_I2C1, ENABLE);    
#endif

#ifdef BSP_USING_I2C2
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

    // Initialize Slave I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable Slave I2C operation */
	I2C_Cmd(LPC_I2C2, ENABLE);    
#endif

    rt_memset((void *)&lpc_i2c2, 0, sizeof(struct lpc_i2c_bus));
    lpc_i2c2.parent.ops = &i2c_ops;
    lpc_i2c2.I2C = LPC_I2C2;
    rt_i2c_bus_device_register(&lpc_i2c2.parent, "i2c2");

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#endif /* RT_USING_I2C_BITOPS */

#endif /* RT_USING_I2C */
