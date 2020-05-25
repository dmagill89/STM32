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


/**
 * GPIO pin numbers
 */
#define GPIO_PIN_0              0
#define GPIO_PIN_1              1
#define GPIO_PIN_2              2
#define GPIO_PIN_3              3
#define GPIO_PIN_4              4
#define GPIO_PIN_5              5
#define GPIO_PIN_6              6
#define GPIO_PIN_7              7
#define GPIO_PIN_8              8
#define GPIO_PIN_9              9
#define GPIO_PIN_10             10
#define GPIO_PIN_11             11
#define GPIO_PIN_12             12
#define GPIO_PIN_13             13
#define GPIO_PIN_14             14
#define GPIO_PIN_15             15

/**
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN            0
#define GPIO_MODE_OUT           1
#define GPIO_MODE_ALTFN         2
#define GPIO_MODE_ANALOG        3
#define GPIO_MODE_IT_FT         4
#define GPIO_MODE_IT_RT         5
#define GPIO_MODE_IT_RFT        6


/**
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP         0
#define GPIO_OP_TYPE_OD         1

/**
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW          0
#define GPIO_SPEED_MEDIUM       1
#define GPIO_SPEED_FAST         2
#define GPIO_SPEED_HIGH         3

/**
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD            0
#define GPIO_PIN_PU             1
#define GPIO_PIN_PD             2


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
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/**
 * IRQ config and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void GPIO_IRQHandling(uint8_t pinNumber);


#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
