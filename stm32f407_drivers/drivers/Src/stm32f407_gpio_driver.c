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
            EXTI->FSTR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
            // 1. configure the rising/falling trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // clear the corresponding RTSR bit
            EXTI->FSTR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // 2. configure the GPIO port selection in SYSCONFIG_EXTICR
        uint8_t extiNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t extiSection = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[extiNumber] = portCode << (extiSection * 4);

        // 3. configure the EXTI interrupt delivery using interrupt mask register IMR
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
 * @param[in]         - the pin we want to toggle
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



/*********************************************************************
 * @fn                - GPIO_IRQInterruptConfig
 *
 * @brief             - This function is used to enabled or clear GPIO interrupts in the NVIC
 *
 * @param[in]         - IRQNumber
 * @param[in]         - Enable of Disable
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis) {

    if (EnOrDis == ENABLE) {

        if (IRQNumber < 31) {

            // program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);

        } else if (IRQNumber > 31 && IRQNumber < 64) {

            // program ISER1 register
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));

        } else if (IRQNumber > 64 && IRQNumber < 96) {

            // program ISER2 register
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
        }
    } else {

        if (IRQNumber < 31) {

            // program ISER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);

        } else if (IRQNumber > 31 && IRQNumber < 64) {

            // program ISER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));

        } else if (IRQNumber > 64 && IRQNumber < 96) {

            // program ISER2 register
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }
    }
}


/*********************************************************************
 * @fn                - GPIO_IRQPriorityConfig
 *
 * @brief             - This function is used to set the priority of an interrupt
 *
 * @param[in]         - IRQNumber
 * @param[in]         - Priority
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

    // first find the ipr register and section offset
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprsection = IRQNumber % 4;
    uint8_t shiftAmount = (8 * iprsection) + 4;

    *(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shiftAmount);
}

/*********************************************************************
 * @fn                - GPIO_IRQHandling
 *
 * @brief             - clear the pr register
 *
 * @param[in]         - pin number
 *
 * @return            - none
 *
 * @Note              -
*/
void GPIO_IRQHandling(uint8_t pinNumber) {

    // clear exti pr register corresponding to the pin number
    if (EXTI->PR & (1 << pinNumber)) {

        // clear
        EXTI->PR |= (1 << pinNumber);
    }
}
