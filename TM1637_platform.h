/**
 **********************************************************************************
 * @file   TM1637_platform.h
 * @author Hossein.M (https://github.com/Hossein-M98)
 * @brief  A sample Platform dependent layer for TM1637 Driver
 **********************************************************************************
 *
 * Copyright (c) 2023 Mahda Embedded System (MIT License)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************
 */
  
/* Define to prevent recursive inclusion ----------------------------------------*/
#ifndef _TM1637_PLATFORM_H_
#define _TM1637_PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ---------------------------------------------------------------------*/
#include "TM1637.h"
#include <stdint.h>


/* Functionality Options --------------------------------------------------------*/
/**
 * @brief  Specify the target platform
 * @note   Uncomment the line below according to the target platform
 */ 
// #define TM1637_PLATFORM_AVR          // ATmega32
// #define TM1637_PLATFORM_STM32        // HAL Functions
// #define TM1637_PLATFORM_ESP32_IDF    // ESP-IDF


#if defined(TM1637_PLATFORM_AVR)
/**
 * @brief  Specify IO Pins of AVR connected to TM1637
 */
#define TM1637_AVR_CLK      8000000UL
#define TM1637_CLK_DDR      DDRA
#define TM1637_CLK_PORT     PORTA
#define TM1637_CLK_NUM      0
#define TM1637_DIO_DDR      DDRA
#define TM1637_DIO_PORT     PORTA
#define TM1637_DIO_PIN      PINA
#define TM1637_DIO_NUM      1

#elif defined(TM1637_PLATFORM_STM32)
/**
 * @brief  Specify IO Pins of STM32 connected to TM1637
 */
#define TM1637_CLK_GPIO     GPIOA
#define TM1637_CLK_PIN      GPIO_PIN_0
#define TM1637_DIO_GPIO     GPIOA
#define TM1637_DIO_PIN      GPIO_PIN_1

#elif defined(TM1637_PLATFORM_ESP32_IDF)
/**
 * @brief  Specify IO Pins of ESP32 connected to TM1637
 */
#define TM1637_CLK_GPIO     GPIO_NUM_0
#define TM1637_DIO_GPIO     GPIO_NUM_1
#endif



/**
 ==================================================================================
                               ##### Functions #####                               
 ==================================================================================
 */

/**
 * @brief  Initialize platform device to communicate TM1637.
 * @param  Handler: Pointer to handler
 * @retval None
 */
void
TM1637_Platform_Init(TM1637_Handler_t *Handler);



#ifdef __cplusplus
}
#endif

#endif //! _TM1637_PLATFORM_H_
