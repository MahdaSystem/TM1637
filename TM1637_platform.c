/**
 **********************************************************************************
 * @file   TM1637_platform.c
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
  
/* Includes ---------------------------------------------------------------------*/
#include "TM1637_platform.h"

#if defined(TM1637_PLATFORM_AVR)
#include <avr/io.h>
#define F_CPU TM1637_AVR_CLK
#include <util/delay.h>
#elif defined(TM1637_PLATFORM_STM32)
#include "main.h"
#elif defined(TM1637_PLATFORM_ESP32_IDF)
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#endif



/**
 ==================================================================================
                           ##### Private Functions #####                           
 ==================================================================================
 */

#if defined(TM1637_PLATFORM_STM32)
void TM1637_SetGPIO_OUT(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
									
void TM1637_SetGPIO_IN_PU(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
#elif defined(TM1637_PLATFORM_ESP32_IDF)
void TM1637_SetGPIO_OUT(gpio_num_t GPIO_Pad)
{
  gpio_reset_pin(GPIO_Pad);
  gpio_set_direction(GPIO_Pad, GPIO_MODE_OUTPUT);
}

void TM1637_SetGPIO_IN_PU(gpio_num_t GPIO_Pad)
{
  gpio_reset_pin(GPIO_Pad);
  gpio_set_direction(GPIO_Pad, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_Pad, GPIO_PULLUP_ONLY);
}
#endif

static void
TM1637_PlatformInit(void)
{
#if defined(TM1637_PLATFORM_AVR)
  TM1637_CLK_DDR |= (1<<TM1637_CLK_NUM);
  TM1637_DIO_DDR |= (1<<TM1637_DIO_NUM);
#elif defined(TM1637_PLATFORM_STM32)
  TM1637_SetGPIO_OUT(TM1637_CLK_GPIO, TM1637_CLK_PIN);
  TM1637_SetGPIO_OUT(TM1637_DIO_GPIO, TM1637_DIO_PIN);
#elif defined(TM1637_PLATFORM_ESP32_IDF)
  TM1637_SetGPIO_OUT(TM1637_CLK_GPIO);
  TM1637_SetGPIO_OUT(TM1637_DIO_GPIO);
#endif
}

static void
TM1637_PlatformDeInit(void)
{
#if defined(TM1637_PLATFORM_AVR)
  TM1637_CLK_DDR &= ~(1<<TM1637_CLK_NUM);
  TM1637_CLK_PORT &= ~(1<<TM1637_CLK_NUM);
  TM1637_DIO_DDR &= ~(1<<TM1637_DIO_NUM);
  TM1637_DIO_PORT &= ~(1<<TM1637_DIO_NUM);
#elif defined(TM1637_PLATFORM_STM32)

#elif defined(TM1637_PLATFORM_ESP32_IDF)
  gpio_reset_pin(TM1637_CLK_GPIO);
  gpio_reset_pin(TM1637_DIO_GPIO);
#endif
}

static void
TM1637_DioConfigOut(void)
{
#if defined(TM1637_PLATFORM_AVR)
  TM1637_DIO_DDR |= (1<<TM1637_DIO_NUM);
#elif defined(TM1637_PLATFORM_STM32)
  TM1637_SetGPIO_OUT(TM1637_DIO_GPIO, TM1637_DIO_PIN);
#elif defined(TM1637_PLATFORM_ESP32_IDF)
  TM1637_SetGPIO_OUT(TM1637_DIO_GPIO);
#endif
}

static void
TM1637_DioConfigIn(void)
{
#if defined(TM1637_PLATFORM_AVR)
  TM1637_DIO_DDR &= ~(1<<TM1637_DIO_NUM);
#elif defined(TM1637_PLATFORM_STM32)
  TM1637_SetGPIO_IN_PU(TM1637_DIO_GPIO, TM1637_DIO_PIN);
#elif defined(TM1637_PLATFORM_ESP32_IDF)
  TM1637_SetGPIO_IN_PU(TM1637_DIO_GPIO);
#endif
}

static void
TM1637_DioWrite(uint8_t Level)
{
#if defined(TM1637_PLATFORM_AVR)
  if (Level)
    TM1637_DIO_PORT |= (1<<TM1637_DIO_NUM);
  else
    TM1637_DIO_PORT &= ~(1<<TM1637_DIO_NUM);
#elif defined(TM1637_PLATFORM_STM32)
  HAL_GPIO_WritePin(TM1637_DIO_GPIO, TM1637_DIO_PIN, Level);
#elif defined(TM1637_PLATFORM_ESP32_IDF)
  gpio_set_level(TM1637_DIO_GPIO, Level);
#endif
}

static uint8_t
TM1637_DioRead(void)
{
  uint8_t Result = 1;
#if defined(TM1637_PLATFORM_AVR)
  Result = (TM1637_DIO_PIN & (1 << TM1637_DIO_NUM)) ? 1 : 0;
#elif defined(TM1637_PLATFORM_STM32)
  Result = HAL_GPIO_ReadPin(TM1637_DIO_GPIO, TM1637_DIO_PIN);
#elif defined(TM1637_PLATFORM_ESP32_IDF)
  Result = gpio_get_level(TM1637_DIO_GPIO);
#endif
  return Result;
}

static void
TM1637_ClkWrite(uint8_t Level)
{
#if defined(TM1637_PLATFORM_AVR)
  if (Level)
    TM1637_CLK_PORT |= (1<<TM1637_CLK_NUM);
  else
    TM1637_CLK_PORT &= ~(1<<TM1637_CLK_NUM);
#elif defined(TM1637_PLATFORM_STM32)
  HAL_GPIO_WritePin(TM1637_CLK_GPIO, TM1637_CLK_PIN, Level);
#elif defined(TM1637_PLATFORM_ESP32_IDF)
  gpio_set_level(TM1637_CLK_GPIO, Level);
#endif
}

static void
TM1637_DelayUs(uint8_t Delay)
{
#if defined(TM1637_PLATFORM_AVR)
  for (; Delay; --Delay)
    _delay_us(1);
#elif defined(TM1637_PLATFORM_STM32)
  for (uint32_t DelayCounter = 0; DelayCounter < 100 * Delay; DelayCounter++)
    DelayCounter = DelayCounter;
#elif defined(TM1637_PLATFORM_ESP32_IDF)
  ets_delay_us(Delay);
#endif
}

/**
 ==================================================================================
                            ##### Public Functions #####                           
 ==================================================================================
 */
 
/**
 * @brief  Initialize platfor device to communicate TM1637.
 * @param  Handler: Pointer to handler
 * @retval None
 */
void
TM1637_Platform_Init(TM1637_Handler_t *Handler)
{
  Handler->PlatformInit = TM1637_PlatformInit;
  Handler->PlatformDeInit = TM1637_PlatformDeInit;
  Handler->DioConfigOut = TM1637_DioConfigOut;
  Handler->DioConfigIn = TM1637_DioConfigIn;
  Handler->DioWrite = TM1637_DioWrite;
  Handler->DioRead = TM1637_DioRead;
  Handler->ClkWrite = TM1637_ClkWrite;
  Handler->DelayUs = TM1637_DelayUs;
}
