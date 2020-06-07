/*
 * led_button_interrupt.c
 *
 *  Created on: Jun 6, 2020
 *      Author: David
 */
#include "stm32f407.h"
#include "stm32f407_gpio_driver.h"
#include<stdint.h>
#include<string.h>

void delay(void) {
    for (uint32_t i = 0; i < 500000 / 2; i++) {}
}

int main(void) {

    GPIO_Handle_t gpioLed, gpioButton;
    memset(&gpioLed, 0, sizeof(gpioLed));
    memset(&gpioButton, 0, sizeof(gpioButton));

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
    gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);

    GPIO_Init(&gpioButton);

    GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    // IRQ configurations
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

    while(1);

    return 0;
}

void EXTI0_IRQHandler(void) {

    delay();

    GPIO_IRQHandling(GPIO_PIN_0);
    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}

