# TM1637 Library
Library for handling TM1637 LED display driver.

## Library Features
-   Support for dimming display

## Hardware Support
It is easy to port this library to any platform. But now it is ready for use in:
- AVR (ATmega32)
- STM32 (HAL)
- ESP32 (esp-idf)

## How To Use
1. Add `TM1637.h` and `TM1637.c` files to your project.  It is optional to use `TM1637_platform.h` and `TM1637_platform.c` files (open and config `TM1637_platform.h` file).
2. Initialize platform-dependent part of handler.
4. Call `TM1637_Init()`.
5. Call `TM1637_ConfigDisplay()` to config display.
6. Call other functions and enjoy.

## Example
<details>
<summary>Using TM1637_platform files</summary>

```c
#include <stdio.h>
#include "TM1637.h"
#include "TM1637_platform.h"

int main(void)
{
  TM1637_Handler_t Handler;

  TM1637_Platform_Init(&Handler);
  TM1637_Init(&Handler);
  TM1637_ConfigDisplay(&Handler, 7, TM1637DisplayStateON);

  while (1)
  {
    // Display the number 8 and Decimal Point in the SEG1 
    TM1637_SetSingleDigit_HEX(&Handler, 8 | TM1637DecimalPoint, 0);
  }

  TM1637_DeInit(&Handler);
  return 0;
}
```
</details>


<details>
<summary>Without using TM1637_platform files (AVR)</summary>

```c
#include <stdio.h>
#include <avr/io.h>
#define F_CPU 8000000
#include <util/delay.h>
#include "TM1637.h"

#define TM1637_DIO_DDR   DDRA
#define TM1637_DIO_PORT  PORTA
#define TM1637_DIO_PIN   PINA
#define TM1637_DIO_NUM   0

#define TM1637_CLK_DDR   DDRA
#define TM1637_CLK_PORT  PORTA
#define TM1637_CLK_NUM   1


static void
TM1637_PlatformInit(void)
{
  TM1637_CLK_DDR |= (1<<TM1637_CLK_NUM);
  TM1637_DIO_DDR |= (1<<TM1637_DIO_NUM);
}

static void
TM1637_PlatformDeInit(void)
{
  TM1637_CLK_DDR &= ~(1<<TM1637_CLK_NUM);
  TM1637_CLK_PORT &= ~(1<<TM1637_CLK_NUM);
  TM1637_DIO_DDR &= ~(1<<TM1637_DIO_NUM);
  TM1637_DIO_PORT &= ~(1<<TM1637_DIO_NUM);
}

static void
TM1637_DioConfigOut(void)
{
  TM1637_DIO_DDR |= (1<<TM1637_DIO_NUM);
}

static void
TM1637_DioConfigIn(void)
{
  TM1637_DIO_DDR &= ~(1<<TM1637_DIO_NUM);
}

static void
TM1637_DioWrite(uint8_t Level)
{
  if (Level)
    TM1637_DIO_PORT |= (1<<TM1637_DIO_NUM);
  else
    TM1637_DIO_PORT &= ~(1<<TM1637_DIO_NUM);
}

static uint8_t
TM1637_DioRead(void)
{
  uint8_t Result = 1;
  Result = (TM1637_DIO_PIN & (1 << TM1637_DIO_NUM)) ? 1 : 0;
  return Result;
}

static void
TM1637_ClkWrite(uint8_t Level)
{
  if (Level)
    TM1637_CLK_PORT |= (1<<TM1637_CLK_NUM);
  else
    TM1637_CLK_PORT &= ~(1<<TM1637_CLK_NUM);
}

static void
TM1637_DelayUs(uint8_t Delay)
{
  for (; Delay; --Delay)
    _delay_us(1);
}


int main(void)
{
  TM1637_Handler_t Handler;

  Handler.PlatformInit = TM1637_PlatformInit;
  Handler.PlatformDeInit = TM1637_PlatformDeInit;
  Handler.DioConfigOut = TM1637_DioConfigOut;
  Handler.DioConfigIn = TM1637_DioConfigIn;
  Handler.DioWrite = TM1637_DioWrite;
  Handler.DioRead = TM1637_DioRead;
  Handler.ClkWrite = TM1637_ClkWrite;
  Handler.DelayUs = TM1637_DelayUs;

  TM1637_Init(&Handler);
  TM1637_ConfigDisplay(&Handler, 7, TM1637DisplayStateON);

  while (1)
  {
    // Display the number 8 and Decimal Point in the SEG1 
    TM1637_SetSingleDigit_HEX(&Handler, 8 | TM1637DecimalPoint, 0);
  }

  TM1637_DeInit(&Handler);
  return 0;
}
```
</details>