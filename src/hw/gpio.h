/*
 * gpio.h
 *
 *  Created on: 2021. 8. 14.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_GPIO_H_
#define SRC_COMMON_HW_INCLUDE_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"


#ifdef _USE_HW_GPIO


#define GPIO_MAX_CH     HW_GPIO_MAX_CH

enum
{
	LED,
	RS_485_DE,
	KEY
};


bool gpioInit(void);
bool gpioPinMode(uint8_t ch, uint8_t mode);
void gpioPinWrite(uint8_t ch, bool value);
bool gpioPinRead(uint8_t ch);
void gpioPinToggle(uint8_t ch);


#endif

#ifdef __cplusplus
}
#endif

#endif /* SRC_COMMON_HW_INCLUDE_GPIO_H_ */
