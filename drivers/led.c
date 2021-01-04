#include "LPC17xx.h"
#include "led.h"

void rt_hw_led_init(void)
{
    LPC_GPIO1->FIODIR |= 1<<18; /* led1:P1.18 */
    LPC_GPIO1->FIODIR |= 1<<20; /* led2:P1.20 */
    LPC_GPIO1->FIODIR |= 1<<21; /* led3:P1.21 */
    LPC_GPIO1->FIODIR |= 1<<23; /* led4:P1.23 */
}

void rt_hw_led_on(rt_uint32_t led)
{
    switch(led)
    {
    case 0: /* P1.18 = 1 */
        LPC_GPIO1->FIOSET = 1<<18;
        break;
    case 1: /* P1.20 = 1 */
        LPC_GPIO1->FIOSET = 1<<20;
        break;
    case 2: /* P1.21 = 1 */
        LPC_GPIO1->FIOSET = 1<<21;
        break;
    case 3: /* P1.23 = 1 */
        LPC_GPIO1->FIOSET = 1<<23;
        break;
    default:
        break;
    }
}

void rt_hw_led_off(rt_uint32_t led)
{
    switch(led)
    {
    case 0: /* P1.18 = 0 */
        LPC_GPIO1->FIOCLR = 1<<18;
        break;
    case 1: /* P1.20 = 0 */
        LPC_GPIO1->FIOCLR = 1<<20;
        break;
    case 2: /* P1.21 = 0 */
        LPC_GPIO1->FIOCLR = 1<<21;
        break;
    case 3: /* P1.23 = 0 */
        LPC_GPIO1->FIOCLR = 1<<23;
        break;
    default:
        break;
    }
}
