/*
 * stm32f407_gpio_driver.h
 *
 *  Created on: May 23, 2020
 *      Author: David
 */

#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include "stm32f407.h"
#include<stdint.h>

/*
 *GPIO pin config structure
 */
typedef struct {
    uint8_t             GPIO_PinNumber;
    uint8_t             GPIO_PinMode;
    uint8_t             GPIO_PinSpeed;
    uint8_t             GPIO_PinPuPdControl;
    uint8_t             GPIO_PinOPType;
    uint8_t             GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/**
 * GPIO handler structure
 */
typedef struct {
    GPIO_RegDef_t       *pGPIOx;
    GPIO_PinConfig_t    GPIO_PinConfig;
} GPIO_Handle_t;

/**************************************************************
 *              API prototypes for GPIO drivers
 **************************************************************/

/**
 * peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis);

/**
 * GPIO Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * GPIO data read/write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, unit8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, unit8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, unit8_t pinNumber);

/**
 * IRQ config and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void GPIO_IRQHandling(unit8_t pinNumber);


#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
