/*
 * modbus.c
 *
 *  Created on: Feb 12, 2025
 *      Author: 왕학승
 */


#include "stm32f1xx_hal.h"
#include <vector>

#define MODBUS_SLAVE_ID  1   // 슬레이브 ID 설정

// 모드버스 기능 코드
#define MODBUS_READ_COILS                 0x01
#define MODBUS_READ_DISCRETE_INPUTS       0x02
#define MODBUS_READ_HOLDING_REGISTERS     0x03
#define MODBUS_READ_INPUT_REGISTERS       0x04
#define MODBUS_WRITE_SINGLE_COIL          0x05
#define MODBUS_WRITE_SINGLE_REGISTER      0x06
#define MODBUS_WRITE_MULTIPLE_COILS       0x0F
#define MODBUS_WRITE_MULTIPLE_REGISTERS   0x10

extern UART_HandleTypeDef huart1;  // UART 핸들 (STM32CubeMX에서 설정 필요)

uint16_t Modbus_CalculateCRC(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Modbus_SendRequest(uint8_t *request, uint16_t length) {
    HAL_UART_Transmit(&huart1, request, length, HAL_MAX_DELAY);
}

bool Modbus_ReceiveResponse(uint8_t *response, uint16_t length) {
    if (HAL_UART_Receive(&huart1, response, length, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    // CRC 검증 (예외 처리)
    uint16_t receivedCRC = (response[length - 1] << 8) | response[length - 2];
    uint16_t calculatedCRC = Modbus_CalculateCRC(response, length - 2);
    return receivedCRC == calculatedCRC;
}

bool Modbus_ReadRegisters(uint8_t slaveID, uint8_t functionCode, uint16_t startAddr, uint16_t numRegs, std::vector<uint8_t> &response) {
    uint8_t request[8];
    request[0] = slaveID;
    request[1] = functionCode;
    request[2] = startAddr >> 8;
    request[3] = startAddr & 0xFF;
    request[4] = numRegs >> 8;
    request[5] = numRegs & 0xFF;

    uint16_t crc = Modbus_CalculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = crc >> 8;

    Modbus_SendRequest(request, 8);
    response.resize(5 + (numRegs * 2));
    return Modbus_ReceiveResponse(response.data(), response.size());
}

bool Modbus_WriteSingleRegister(uint8_t slaveID, uint16_t regAddr, uint16_t value) {
    uint8_t request[8];
    request[0] = slaveID;
    request[1] = MODBUS_WRITE_SINGLE_REGISTER;
    request[2] = regAddr >> 8;
    request[3] = regAddr & 0xFF;
    request[4] = value >> 8;
    request[5] = value & 0xFF;

    uint16_t crc = Modbus_CalculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = crc >> 8;

    Modbus_SendRequest(request, 8);
    uint8_t response[8];
    return Modbus_ReceiveResponse(response, 8);
}

bool Modbus_WriteMultipleRegisters(uint8_t slaveID, uint16_t startAddr, std::vector<uint16_t> values) {
    uint16_t numRegs = values.size();
    uint8_t request[9 + (numRegs * 2)];
    request[0] = slaveID;
    request[1] = MODBUS_WRITE_MULTIPLE_REGISTERS;
    request[2] = startAddr >> 8;
    request[3] = startAddr & 0xFF;
    request[4] = numRegs >> 8;
    request[5] = numRegs & 0xFF;
    request[6] = numRegs * 2;

    for (size_t i = 0; i < numRegs; i++) {
        request[7 + (i * 2)] = values[i] >> 8;
        request[8 + (i * 2)] = values[i] & 0xFF;
    }

    uint16_t crc = Modbus_CalculateCRC(request, 7 + (numRegs * 2));
    request[7 + (numRegs * 2)] = crc & 0xFF;
    request[8 + (numRegs * 2)] = crc >> 8;

    Modbus_SendRequest(request, sizeof(request));
    uint8_t response[8];
    return Modbus_ReceiveResponse(response, 8);
}
