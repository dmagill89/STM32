/*
 * stm32f407.h
 *
 *  Created on: May 17, 2020
 *      Author: David
 */

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_

/**
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x2001C000U
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM						SRAM1_BASEADDR

/**
 * AHBx and APBx Bus peripheral base addresses
 */

#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x50000000U


#endif /* INC_STM32F407_H_ */
