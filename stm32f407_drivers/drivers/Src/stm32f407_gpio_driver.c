/*
 * stm32f407_gpio.c
 *
 *  Created on: May 23, 2020
 *      Author: David
 */

#include "stm32f407_gpio_driver.h"
#include <stdint.h>

/**
 * peripheral clock setup
 */

/*********************************************************************
 * @fn                - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis) {
    if (EnOrDis == ENABLE) {

        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();
        } else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_EN();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        } else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_EN();
        }
    } else {

        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DIS();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DIS();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DIS();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DIS();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_DIS();
        } else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_DIS();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_DIS();
        } else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_DIS();
        }
    }
}

/**
 * GPIO Init and Deinit
 */

/*********************************************************************
 * @fn                - GPIO_Init
 *
 * @brief             - This function enables a GPIO port
 *
 * @param[in]         - gpio handler struct
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
    uint32_t temp = 0;

    // configure the mode of the gpio pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        // non interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bits
        pGPIOHandle->pGPIOx->MODER |= temp;
    } else {
        // interrupt mode

        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
            // 1. configure the falling trigger
            EXTI->FSTR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // clear the corresponding RTSR bit
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
            // 1. configure the rising trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // clear the corresponding RTSR bit
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
            // 1. configure the rising/falling trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // clear the corresponding RTSR bit
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // 2. configure the GPIO port selection in SYSCONFIG_EXTICR

        // 3. configure the EXTI interrupt delivery using IMR
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    temp = 0;

    // configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear bits
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    // configure pull-up/pull-down settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bits
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;

    // configure the output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear bits
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    // configure the alt functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN) {
        uint32_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // clear bits
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }

}

/*********************************************************************
 * @fn                - GPIO_DeInit
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB) {
        GPIOB_REG_RESET();
    } else if (pGPIOx == GPIOC) {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOD) {
        GPIOD_REG_RESET();
    } else if (pGPIOx == GPIOE) {
        GPIOE_REG_RESET();
    } else if (pGPIOx == GPIOF) {
        GPIOF_REG_RESET();
    } else if (pGPIOx == GPIOG) {
        GPIOG_REG_RESET();
    } else if (pGPIOx == GPIOH) {
        GPIOH_REG_RESET();
    } else if (pGPIOx == GPIOI) {
        GPIOI_REG_RESET();
    }
}

/**
 * GPIO data read/write
 */

/*********************************************************************
 * @fn                - GPIO_ReadFromInputPin
 *
 * @brief             - This function returns the value of the input data register for a GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - the pin number that we want to read
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    uint8_t value;

    value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);

    return value;
}

/*********************************************************************
 * @fn                - GPIO_ReadFromInputPort
 *
 * @brief             - This function returns the value of the input port for a GPIO
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            -  uint16_t
 *
 * @Note              -  none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
    uint16_t value;

    value = pGPIOx->IDR;

    return value;
}

/*********************************************************************
 * @fn                - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes a value to the input data register for a GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - the pin number that we want to read
 * @param[in]         - the value we want to write to the pin
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
        uint8_t value) {

    if (value == GPIO_PIN_SET) {
        // write 1 to the output data register at the corresponding pin number
        pGPIOx->ODR |= (1 << PinNumber);
    } else {
        // write 0 to the output data register at the corresponding pin number
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/*********************************************************************
 * @fn                - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes a value to the input data register for a GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - the value we want to write to the port
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
    pGPIOx->ODR = value;
}

/*********************************************************************
 * @fn                - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the input data register for a GPIO pin
 *
 * @param[in]         - base address of the gpio peripheral
 *  @param[in]         - the pin we want to toggle
 *
 * @return            -  0 or 1
 *
 * @Note              -  none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    pGPIOx->ODR ^= (1 << PinNumber);
}

/**
 * IRQ config and ISR handling
 */
//void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
//void GPIO_IRQHandling(unit8_t pinNumber);
