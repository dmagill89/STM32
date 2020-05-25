/*
 * led_button.c
 *
 * toggle LED4 using the on board user button
 *
 *  Created on: May 25, 2020
 *      Author: David
 */
#include "stm32f407.h"
#include "stm32f407_gpio_driver.h"
#include<stdint.h>

void delay(void) {
    for (uint32_t i = 0; i < 500000 / 2; i++) {}
}

int main(void) {

    GPIO_Handle_t gpioLed, gpioButton;

    // GPIO led4 setup, led4 is on pin 12 of port D
    gpioLed.pGPIOx = GPIOD;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Init(&gpioLed);

    // GPIO button setup, button is on pin 0 of port A
    gpioButton.pGPIOx = GPIOA;
    gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
    gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);

    GPIO_Init(&gpioButton);

    while (1) {

        if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) == ENABLE) {
            delay();
            GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
        }
    }

    return 0;
}
