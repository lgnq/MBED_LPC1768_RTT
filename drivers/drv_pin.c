/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2018-11-06     balanceTWK        first version
 * 2019-04-23     WillianChan       Fix GPIO serial number disorder
 * 2020-06-16     thread-liu        add lpc17xxMP1
 * 2020-09-01     thread-liu        add GPIOZ
 * 2020-09-18     geniusgogo        optimization design pin-index algorithm
 */

#include <rtthread.h>

#include <board.h>
#include "drv_pin.h"
#include <drivers/pin.h>
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"

#ifdef RT_USING_PIN

#define PIN_NUM(port, no) ((port << 5) | (no))
#define PIN_PORT(pin) ((uint8_t)((pin) >> 5))
#define PIN_NO(pin) ((uint8_t)((pin) & 0x1Fu))

// static const struct pin_irq_map pin_irq_map[] =
// {
//     {GPIO_PIN_0, EXTI0_IRQn},
//     {GPIO_PIN_1, EXTI1_IRQn},
//     {GPIO_PIN_2, EXTI2_IRQn},
//     {GPIO_PIN_3, EXTI3_IRQn},
//     {GPIO_PIN_4, EXTI4_IRQn},
//     {GPIO_PIN_5, EXTI9_5_IRQn},
//     {GPIO_PIN_6, EXTI9_5_IRQn},
//     {GPIO_PIN_7, EXTI9_5_IRQn},
//     {GPIO_PIN_8, EXTI9_5_IRQn},
//     {GPIO_PIN_9, EXTI9_5_IRQn},
//     {GPIO_PIN_10, EXTI15_10_IRQn},
//     {GPIO_PIN_11, EXTI15_10_IRQn},
//     {GPIO_PIN_12, EXTI15_10_IRQn},
//     {GPIO_PIN_13, EXTI15_10_IRQn},
//     {GPIO_PIN_14, EXTI15_10_IRQn},
//     {GPIO_PIN_15, EXTI15_10_IRQn},
// };

// static struct rt_pin_irq_hdr pin_irq_hdr_tab[] =
// {
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
//     {-1, 0, RT_NULL, RT_NULL},
// };
// static uint32_t pin_irq_enable_mask = 0;

// #define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

static rt_base_t lpc17xx_pin_get(const char *name)
{
    rt_base_t pin = 0;
    int hw_port_num, hw_pin_num = 0;
    int i, name_len;

    name_len = rt_strlen(name);

    if ((name_len < 4) || (name_len >= 6))
    {
        return -RT_EINVAL;
    }
    if ((name[0] != 'P') || (name[2] != '.'))
    {
        return -RT_EINVAL;
    }

    if ((name[1] >= '0') && (name[1] <= '4'))
    {
        hw_port_num = (int)(name[1] - '0');
    }
    else
    {
        return -RT_EINVAL;
    }

    for (i = 3; i < name_len; i++)
    {
        hw_pin_num *= 10;
        hw_pin_num += name[i] - '0';
    }

    pin = PIN_NUM(hw_port_num, hw_pin_num);

    return pin;
}

static void lpc17xx_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{   
    uint8_t port_num;
    uint32_t bit_value;

    port_num  = PIN_PORT(pin); 
    bit_value = PIN_NO(pin);

    if (value == PIN_HIGH)
        GPIO_SetValue(port_num, 1<<bit_value);
    else
        GPIO_ClearValue(port_num, 1<<bit_value);
}

static int lpc17xx_pin_read(rt_device_t dev, rt_base_t pin)
{
    int value = PIN_LOW;

    uint8_t port_num;
    uint32_t bit_value;

    port_num  = PIN_PORT(pin); 
    bit_value = PIN_NO(pin);

    value = GPIO_ReadValue(port_num);
    // rt_kprintf("0x%x\r\n", value);
    if (value & 1<<bit_value)
        value = PIN_HIGH;
    else
        value = PIN_LOW;

    return value;
}

static void lpc17xx_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    uint8_t port_num;
    uint32_t bit_value;

    PINSEL_CFG_Type PinCfg;

    port_num  = PIN_PORT(pin); 
    bit_value = PIN_NO(pin);

    PinCfg.Funcnum = 0;
    PinCfg.Pinnum = bit_value;
    PinCfg.Portnum = port_num;

    if (mode == PIN_MODE_OUTPUT)
    {
        /* output setting */
        GPIO_SetDir(port_num, 1<<bit_value, 1);

        PinCfg.OpenDrain = 0;
        PinCfg.Pinmode = 0;
    }
    else if (mode == PIN_MODE_INPUT)
    {
        /* input setting: not pull. */
        GPIO_SetDir(port_num, 1<<bit_value, 0);
    
        PinCfg.OpenDrain = 0;
        PinCfg.Pinmode = 0;    
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        /* input setting: pull up. */
        GPIO_SetDir(port_num, 1<<bit_value, 0);

        PinCfg.OpenDrain = 0;
        PinCfg.Pinmode = 0;        
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        /* input setting: pull down. */
        GPIO_SetDir(port_num, 1<<bit_value, 0);

        PinCfg.OpenDrain = 0;
        PinCfg.Pinmode = 2;        

    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        /* output setting: od. */
        GPIO_SetDir(port_num, 1<<bit_value, 1);

        PinCfg.OpenDrain = 1;
        PinCfg.Pinmode = 0;        
    }

    PINSEL_ConfigPin(&PinCfg);        
}

// rt_inline const struct pin_irq_map *get_pin_irq_map(uint32_t pinbit)
// {
//     // rt_int32_t mapindex = bit2bitno(pinbit);
//     // if (mapindex < 0 || mapindex >= ITEM_NUM(pin_irq_map))
//     // {
//     //     return RT_NULL;
//     // }
//     // return &pin_irq_map[mapindex];
// };

static rt_err_t lpc17xx_pin_attach_irq(struct rt_device *device, rt_int32_t pin, rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    // rt_base_t level;
    // rt_int32_t irqindex = -1;

    // if (PIN_PORT(pin) >= PIN_STPORT_MAX)
    // {
    //     return -RT_ENOSYS;
    // }

    // irqindex = bit2bitno(PIN_STPIN(pin));
    // if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    // {
    //     return RT_ENOSYS;
    // }

    // level = rt_hw_interrupt_disable();
    // if (pin_irq_hdr_tab[irqindex].pin == pin &&
    //     pin_irq_hdr_tab[irqindex].hdr == hdr &&
    //     pin_irq_hdr_tab[irqindex].mode == mode &&
    //     pin_irq_hdr_tab[irqindex].args == args)
    // {
    //     rt_hw_interrupt_enable(level);
    //     return RT_EOK;
    // }
    // if (pin_irq_hdr_tab[irqindex].pin != -1)
    // {
    //     rt_hw_interrupt_enable(level);
    //     return RT_EBUSY;
    // }
    // pin_irq_hdr_tab[irqindex].pin = pin;
    // pin_irq_hdr_tab[irqindex].hdr = hdr;
    // pin_irq_hdr_tab[irqindex].mode = mode;
    // pin_irq_hdr_tab[irqindex].args = args;
    // rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t lpc17xx_pin_dettach_irq(struct rt_device *device, rt_int32_t pin)
{
    // rt_base_t level;
    // rt_int32_t irqindex = -1;

    // if (PIN_PORT(pin) >= PIN_STPORT_MAX)
    // {
    //     return -RT_ENOSYS;
    // }

    // irqindex = bit2bitno(PIN_STPIN(pin));
    // if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    // {
    //     return RT_ENOSYS;
    // }

    // level = rt_hw_interrupt_disable();
    // if (pin_irq_hdr_tab[irqindex].pin == -1)
    // {
    //     rt_hw_interrupt_enable(level);
    //     return RT_EOK;
    // }
    // pin_irq_hdr_tab[irqindex].pin = -1;
    // pin_irq_hdr_tab[irqindex].hdr = RT_NULL;
    // pin_irq_hdr_tab[irqindex].mode = 0;
    // pin_irq_hdr_tab[irqindex].args = RT_NULL;
    // rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t lpc17xx_pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint32_t enabled)
{
//     const struct pin_irq_map *irqmap;
//     rt_base_t level;
//     rt_int32_t irqindex = -1;
//     GPIO_InitTypeDef GPIO_InitStruct;

//     if (PIN_PORT(pin) >= PIN_STPORT_MAX)
//     {
//         return -RT_ENOSYS;
//     }

//     if (enabled == PIN_IRQ_ENABLE)
//     {
//         irqindex = bit2bitno(PIN_STPIN(pin));
//         if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
//         {
//             return RT_ENOSYS;
//         }

//         level = rt_hw_interrupt_disable();

//         if (pin_irq_hdr_tab[irqindex].pin == -1)
//         {
//             rt_hw_interrupt_enable(level);
//             return RT_ENOSYS;
//         }

//         irqmap = &pin_irq_map[irqindex];

//         /* Configure GPIO_InitStructure */
//         GPIO_InitStruct.Pin = PIN_STPIN(pin);
//         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//         switch (pin_irq_hdr_tab[irqindex].mode)
//         {
//         case PIN_IRQ_MODE_RISING:
//             GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//             GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//             break;
//         case PIN_IRQ_MODE_FALLING:
//             GPIO_InitStruct.Pull = GPIO_PULLUP;
//             GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//             break;
//         case PIN_IRQ_MODE_RISING_FALLING:
//             GPIO_InitStruct.Pull = GPIO_NOPULL;
//             GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//             break;
//         }
//         HAL_GPIO_Init(PIN_STPORT(pin), &GPIO_InitStruct);

//         HAL_NVIC_SetPriority(irqmap->irqno, 5, 0);
//         HAL_NVIC_EnableIRQ(irqmap->irqno);
//         pin_irq_enable_mask |= irqmap->pinbit;

//         rt_hw_interrupt_enable(level);
//     }
//     else if (enabled == PIN_IRQ_DISABLE)
//     {
//         irqmap = get_pin_irq_map(PIN_STPIN(pin));
//         if (irqmap == RT_NULL)
//         {
//             return RT_ENOSYS;
//         }

//         level = rt_hw_interrupt_disable();

//         HAL_GPIO_DeInit(PIN_STPORT(pin), PIN_STPIN(pin));

//         pin_irq_enable_mask &= ~irqmap->pinbit;
// #if defined(SOC_SERIES_lpc17xxF0) || defined(SOC_SERIES_lpc17xxG0)
//         if ((irqmap->pinbit >= GPIO_PIN_0) && (irqmap->pinbit <= GPIO_PIN_1))
//         {
//             if (!(pin_irq_enable_mask & (GPIO_PIN_0 | GPIO_PIN_1)))
//             {
//                 HAL_NVIC_DisableIRQ(irqmap->irqno);
//             }
//         }
//         else if ((irqmap->pinbit >= GPIO_PIN_2) && (irqmap->pinbit <= GPIO_PIN_3))
//         {
//             if (!(pin_irq_enable_mask & (GPIO_PIN_2 | GPIO_PIN_3)))
//             {
//                 HAL_NVIC_DisableIRQ(irqmap->irqno);
//             }
//         }
//         else if ((irqmap->pinbit >= GPIO_PIN_4) && (irqmap->pinbit <= GPIO_PIN_15))
//         {
//             if (!(pin_irq_enable_mask & (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
//                                          GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)))
//             {
//                 HAL_NVIC_DisableIRQ(irqmap->irqno);
//             }
//         }
//         else
//         {
//             HAL_NVIC_DisableIRQ(irqmap->irqno);
//         }
// #else
//         if ((irqmap->pinbit >= GPIO_PIN_5) && (irqmap->pinbit <= GPIO_PIN_9))
//         {
//             if (!(pin_irq_enable_mask & (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9)))
//             {
//                 HAL_NVIC_DisableIRQ(irqmap->irqno);
//             }
//         }
//         else if ((irqmap->pinbit >= GPIO_PIN_10) && (irqmap->pinbit <= GPIO_PIN_15))
//         {
//             if (!(pin_irq_enable_mask & (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)))
//             {
//                 HAL_NVIC_DisableIRQ(irqmap->irqno);
//             }
//         }
//         else
//         {
//             HAL_NVIC_DisableIRQ(irqmap->irqno);
//         }
// #endif
//         rt_hw_interrupt_enable(level);
//     }
//     else
//     {
//         return -RT_ENOSYS;
//     }

    return RT_EOK;
}
const static struct rt_pin_ops lpc17xx_pin_ops =
{
    lpc17xx_pin_mode,
    lpc17xx_pin_write,
    lpc17xx_pin_read,
    lpc17xx_pin_attach_irq,
    lpc17xx_pin_dettach_irq,
    lpc17xx_pin_irq_enable,
    lpc17xx_pin_get,
};

void EXTI0_IRQHandler(void)
{
    rt_interrupt_enter();
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    rt_interrupt_leave();
}

void EXTI1_IRQHandler(void)
{
    rt_interrupt_enter();
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    rt_interrupt_leave();
}

void EXTI2_IRQHandler(void)
{
    rt_interrupt_enter();
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    rt_interrupt_leave();
}

void EXTI3_IRQHandler(void)
{
    rt_interrupt_enter();
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    rt_interrupt_leave();
}

void EXTI4_IRQHandler(void)
{
    rt_interrupt_enter();
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
    rt_interrupt_leave();
}

void EXTI9_5_IRQHandler(void)
{
    rt_interrupt_enter();
    rt_interrupt_leave();
}

void EXTI15_10_IRQHandler(void)
{
    rt_interrupt_enter();

    rt_interrupt_leave();
}

int rt_hw_pin_init(void)
{
    return rt_device_pin_register("pin", &lpc17xx_pin_ops, RT_NULL);
}

#endif /* RT_USING_PIN */
