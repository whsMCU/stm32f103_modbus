/*
 * uart.h
 *
 *  Created on: 2020. 12. 8.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_UART_H_
#define SRC_COMMON_HW_INCLUDE_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"

#define UART_MAX_CH         HW_UART_MAX_CH

typedef enum {
    BAUD_AUTO = 0,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
    BAUD_400000,
    BAUD_460800,
    BAUD_500000,
    BAUD_921600,
    BAUD_1000000,
    BAUD_1500000,
    BAUD_2000000,
    BAUD_2470000,
    BAUD_COUNT
} baudRate_e;

#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

extern const uint32_t baudRates[];

bool     uartInit(void);
bool     uartOpen(uint8_t ch, uint32_t baud);
bool uartIsConnected(uint8_t ch);
uint32_t uartAvailable(uint8_t ch);
bool uartTxBufEmpty(uint8_t ch);
uint32_t uartTotalTxBytesFree(uint8_t ch);
void waitForSerialPortToFinishTransmitting(uint8_t ch);
uint8_t  uartRead(uint8_t ch);
uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length);
uint32_t uartWriteIT(uint8_t ch, uint8_t *p_data, uint32_t length);
uint32_t uartWriteDMA(uint8_t ch, uint8_t *p_data, uint32_t length);
void serialPrint(uint8_t channel, const char *str);
uint32_t uartPrintf(uint8_t ch, char *fmt, ...);
uint32_t uartPrintf_IT(uint8_t ch, char *fmt, ...);
uint32_t uartGetBaud(uint8_t ch);
bool uartSetBaud(uint8_t ch, uint32_t baud);
baudRate_e lookupBaudRateIndex(uint32_t baudRate);

#ifdef __cplusplus
}
#endif

#endif /* SRC_COMMON_HW_INCLUDE_UART_H_ */
