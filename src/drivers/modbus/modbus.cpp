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
  _defaultId = 1;
  _RxSize = 0;
  _availableForWrite_flag = true;
  _c_state = MODBUS_IDLE;
  _RxOffset = 0;
  _RxChecksum = 0;
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

static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }

  return crc;
}

unsigned int word(unsigned char h, unsigned char l) { return (h << 8) | l; }

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
      _function = MODBUS_FC_READ_HOLDING_REGISTERS;
      _length = 8;
      _availableForWrite_flag = false;
      gpioPinWrite(_de_pin, _DEF_HIGH);
      uartWriteIT(_serial, (uint8_t *)_TxData, _length);
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

uint8_t Modbus::passer()
{
	uint8_t c;

	while(uartAvailable(_serial)>0)
	{
		c = uartRead(_serial);
		if(_c_state == MODBUS_IDLE){
			_c_state = (c==_slaveId) ? MODBUS_ID : MODBUS_IDLE;
			_RxIndex = 0;
			_RxData[_RxIndex++] = c;
		} else if(_c_state == MODBUS_ID){
			_c_state = (c==_function) ? MODBUS_FUNCTION : MODBUS_IDLE;
			_RxData[_RxIndex++] = c;
		} else if(_c_state == MODBUS_FUNCTION){
			if(c > 255){
				_c_state = MODBUS_IDLE;
			}
			_RxData[_RxIndex++] = c;
			_RxSize = c;
			_RxOffset = 0;
			_RxChecksum = 0;
			_c_state = MODBUS_SIZE;
		} else if(_c_state == MODBUS_SIZE){
			_RxData[_RxIndex++] = c;
			_RxOffset++;
			_c_state = MODBUS_DATA;
		}else if(_c_state == MODBUS_DATA && _RxOffset < _RxSize+5){
			_RxData[_RxIndex++] = c;
			_RxOffset++;
		}else if(_c_state == MODBUS_DATA && _RxOffset >= _RxSize+5){
			// calculate CRC
			_RxChecksum = 0xFFFF;
			for(int i = 0; i < (_RxIndex - 2); i++)
			{
				_RxChecksum = crc16_update(_RxChecksum, _RxData[i]);
			}

			// verify CRC
			if(_RxChecksum == (_RxData[_RxIndex - 2] | _RxData[_RxIndex - 1]<<8))
			{
				evaluateCommand();
			}
			_c_state = MODBUS_IDLE;
			_RxIndex = 0;
		}
	}


		if(!_RxStatus)
		{
			switch(_RxData[1])
			{
				case MODBUS_FC_READ_HOLDING_REGISTERS:
				{
	        // load bytes into word; response bytes are ordered H, L, H, L, ...
	        for (int i = 0; i < (_RxData[2] >> 1); i++)
	        {
	          if (i < MaxBufferSize)
	          {
	            _u16ResponseBuffer[i] = word(_RxData[2 * i + 3], _RxData[2 * i + 4]);
	          }

	          _u8ResponseBufferLength = i;
	        }
	        break;
				}
			}
		}

	return _c_state;
}

void Modbus::evaluateCommand(void) {
	 switch(_function){
		 case MODBUS_FC_READ_HOLDING_REGISTERS:
       // load bytes into word; response bytes are ordered H, L, H, L, ...
       for (int i = 0; i < (_RxData[2] >> 1); i++)
       {
         if (i < MaxBufferSize)
         {
           _u16ResponseBuffer[i] = word(_RxData[2 * i + 3], _RxData[2 * i + 4]);
         }

         _u8ResponseBufferLength = i;
       }
			 break;

	 }
}
