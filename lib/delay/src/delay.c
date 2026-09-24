#include "delay.h"

void delay_init(u8 system_clock_mhz)
{
  (void)system_clock_mhz;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(u32 microseconds)
{
  uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  uint32_t start;
  uint32_t wait_cycles;

  if ((microseconds == 0U) || (cycles_per_us == 0U)) {
    return;
  }

  wait_cycles = microseconds * cycles_per_us;
  start = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - start) < wait_cycles) {
  }
}

void delay_ms(u16 milliseconds)
{
  HAL_Delay((uint32_t)milliseconds);
}
