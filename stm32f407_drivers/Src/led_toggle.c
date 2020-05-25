/*
 * led_toggle.c
 *
 * toggle LED4 using the GPIO drivers built in this project
 *
 *  Created on: May 25, 2020
 *      Author: David
 */
#include "stm32f407.h"
#include "stm32f407_gpio_driver.h"
#include<stdint.h>

void delay(void) {
    for (uint32_t i = 0; i < 500000 ; i++) {}
}

int main(void) {

    GPIO_Handle_t gpioLed;

    gpioLed.pGPIOx = GPIOD;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Init(&gpioLed);

    while(1) {
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
        delay();
    }

    return 0;
}
