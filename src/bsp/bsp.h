/*
 * bsp.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define _USE_LOG_PRINT    1

#if _USE_LOG_PRINT
#define logPrintf(fmt, ...)     printf(fmt, ##__VA_ARGS__)
#else
#define logPrintf(fmt, ...)
#endif

void delay(uint32_t ms);
uint32_t millis(void);
uint32_t micros(void);
void delayMicroseconds(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* SRC_BSP_BSP_H_ */
