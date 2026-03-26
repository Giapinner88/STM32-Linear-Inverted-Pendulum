#ifndef __LED_BLINK_TEST_H
#define __LED_BLINK_TEST_H

#include "main.h"

/* Function to initialize and blink LED on PA4 */
void LED_Blink_Init(void);

/* Function to blink LED with custom interval (in ms) */
void LED_Blink(uint16_t interval_ms);

/* Function to blink LED N times */
void LED_Blink_N_Times(uint16_t times, uint16_t interval_ms);

#endif /* __LED_BLINK_TEST_H */
