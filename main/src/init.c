#include "slamdeck.h"
#include "led.h"
#include "button.h"
#include "i2c.h"


void sys_init()
{
    led_init();
    button_init();
    //i2c_init();
}
