/*
 * modbus.cpp
 *
 *  Created on: Feb 13, 2025
 *      Author: WANG
 */

#include <Modbus.h>


Modbus::Modbus(void)
{
	_id = 1;
	_RxSize = 0;
  _availableForWrite_flag = true;
}


void Modbus::begin(uint8_t serial, uint8_t de_pin)
{
  _serial = serial;
  _de_pin = de_pin;
}

static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

static int send_msg_pre(uint8_t *req, int req_length)
{
    uint16_t crc = crc16(req, req_length);
    req[req_length++] = crc >> 8;
    req[req_length++] = crc & 0x00FF;

    return req_length;
}


uint8_t read_holding_registers(uint8_t id, uint8_t address, uint8_t nb, uint8_t *req)
{
	uint8_t result = 0;
	req[0] = id;
	req[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
	req[2] = highByte(address);
	req[3] = lowByte(address);
	req[4] = highByte(nb);
	req[5] = lowByte(nb);
	send_msg_pre(req, 6);

	result = 1;
	return result;
}

uint8_t Modbus::request(uint8_t slaveId, uint8_t funtion, uint8_t address, uint8_t nb)
{
  int result = -1;

  if (slaveId >= 0 && slaveId <= 247)
  {
  	_slaveId = slaveId;
  }else
  {
  	return 0;
  }

  if(_availableForWrite_flag == false)
  {
  	return 0;
  }

  switch (funtion) {
    case MODBUS_FC_READ_HOLDING_REGISTERS:
      result = read_holding_registers(_slaveId, address, nb, _TxData);
      _length = 8;
      _availableForWrite_flag = false;
      gpioPinWrite(_de_pin, _DEF_HIGH);
      uartWriteIT(_serial, _TxData, _length);
      break;

    default:
      break;
  }

  if (result == -1) {
    return 0;
  }

  return nb;
}

void Modbus::sended(void)
{
	_availableForWrite_flag = true;
}

uint8_t Modbus::get_de_pin(void)
{
	return _de_pin;
}

uint8_t Modbus::passer(uint8_t c)
{
	_RxStatus = Success;

	_RxData[_RxSize++] = c;

	if(_RxSize == 5)
	{
		if(_RxData[0] != _slaveId)
		{
			_RxStatus = InvalidSlaveID;
			return 0;
		}

		if((_RxData[1] & 0x7F) != _funtion)
		{
			_RxStatus = InvalidFunction;
			return 0;
		}

		if(bitRead(_RxData[1], 7))
		{
			_RxStatus = _RxData[2];
			return 0;
		}

	}
}
