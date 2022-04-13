#include "slamdeck.h"
#include "led.h"
#include "i2c.h"


void sys_init()
{
    led_init();
    //i2c_init();
}
