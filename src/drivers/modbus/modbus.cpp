/*
 * modbus.cpp
 *
 *  Created on: Feb 13, 2025
 *      Author: WANG
 */

#include <Modbus.h>

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

Modbus::Modbus(void)
{
  _RxSize = 0;
  _availableForWrite_flag = true;
  _c_state = MODBUS_IDLE;
  _RxOffset = 0;
  _RxChecksum = 0;
}


void Modbus::begin(uint8_t serial, uint8_t id, uint8_t de_pin)
{
  _serial = serial;
  _id			= id;
  _de_pin = de_pin;
}

uint8_t Modbus::read_coils(uint8_t slaveId, uint16_t address, uint16_t qty)
{
	uint8_t idx = 0;
	uint8_t size = 0;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_READ_COILS;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(qty);
	_TxData[idx++] = lowByte(qty);
  size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_READ_COILS;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::read_discrete_inputs(uint8_t slaveId, uint16_t address, uint16_t qty)
{
	uint8_t idx = 0;
	uint8_t size = 0;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_READ_DISCRETE_INPUTS;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(qty);
	_TxData[idx++] = lowByte(qty);
	size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_READ_DISCRETE_INPUTS;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::read_holding_registers(uint8_t slaveId, uint16_t address, uint16_t qty)
{
	uint8_t idx = 0;
	uint8_t size = 0;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_READ_HOLDING_REGISTERS;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(qty);
	_TxData[idx++] = lowByte(qty);
	size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_READ_HOLDING_REGISTERS;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::read_input_registers(uint8_t slaveId, uint16_t address, uint16_t qty)
{
	uint8_t idx = 0;
	uint8_t size = 0;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_READ_INPUT_REGISTERS;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(qty);
	_TxData[idx++] = lowByte(qty);
	size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_READ_INPUT_REGISTERS;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::write_single_coil(uint8_t slaveId, uint16_t address, uint16_t status)
{
	uint8_t idx = 0;
	uint8_t size = 0;
	uint16_t qty = (status ? 0xFF00 : 0x0000);

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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_WRITE_SINGLE_COIL;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(qty);
	_TxData[idx++] = lowByte(qty);

  size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_WRITE_SINGLE_COIL;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::write_single_register(uint8_t slaveId, uint16_t address, uint16_t value)
{
	uint8_t idx = 0;
	uint8_t size = 0;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_WRITE_SINGLE_REGISTER;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(value);
	_TxData[idx++] = lowByte(value);
	size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_WRITE_SINGLE_REGISTER;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::write_multiple_coils(uint8_t slaveId, uint16_t address, uint16_t *value, uint16_t qty)
{
	uint8_t idx = 0;
	uint8_t size = 0;
	uint8_t u8Qty;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_WRITE_MULTIPLE_COILS;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(qty);
	_TxData[idx++] = lowByte(qty);
  u8Qty = (qty % 8) ? ((qty >> 3) + 1) : (qty >> 3);
  _TxData[6] = u8Qty;
  for (int i = 0; i < u8Qty; i++)
  {
    switch(i % 2)
    {
      case 0: // i is even
      	_TxData[idx++] = lowByte(value[i >> 1]);
        break;

      case 1: // i is odd
      	_TxData[idx++] = highByte(value[i >> 1]);
        break;
    }
  }

  size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_WRITE_MULTIPLE_COILS;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::write_multiple_registers(uint8_t slaveId, uint16_t address, uint16_t *value, uint16_t qty)
{
	uint8_t idx = 0;
	uint8_t size = 0;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(qty);
	_TxData[idx++] = lowByte(qty);
	_TxData[idx++] = lowByte(qty)<<1;

  for (int i = 0; i < lowByte(qty); i++)
  {
  	_TxData[idx++] = highByte(value[i]);
  	_TxData[idx++] = lowByte(value[i]);
  }

  size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::write_mask_register(uint8_t slaveId, uint16_t address, uint16_t AndMask, uint16_t OrMask)
{
	uint8_t idx = 0;
	uint8_t size = 0;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_MASK_WRITE_REGISTER;
	_TxData[idx++] = highByte(address);
	_TxData[idx++] = lowByte(address);
	_TxData[idx++] = highByte(AndMask);
	_TxData[idx++] = lowByte(AndMask);
	_TxData[idx++] = highByte(OrMask);
	_TxData[idx++] = lowByte(OrMask);

  size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_MASK_WRITE_REGISTER;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
}

uint8_t Modbus::write_read_registers(uint8_t slaveId, uint16_t ReadAddress, uint16_t ReadQty, uint16_t WriteAddress, uint16_t *value, uint16_t WriteQty)
{
	uint8_t idx = 0;
	uint8_t size = 0;
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

	_TxData[idx++] = slaveId;
	_TxData[idx++] = MODBUS_FC_WRITE_AND_READ_REGISTERS;
	_TxData[idx++] = highByte(ReadAddress);
	_TxData[idx++] = lowByte(ReadAddress);
	_TxData[idx++] = highByte(ReadQty);
	_TxData[idx++] = lowByte(ReadQty);
	_TxData[idx++] = highByte(WriteAddress);
	_TxData[idx++] = lowByte(WriteAddress);
	_TxData[idx++] = highByte(WriteQty);
	_TxData[idx++] = lowByte(WriteQty);
	_TxData[idx++] = lowByte(WriteQty<<1);
  for (int i = 0; i < lowByte(WriteQty); i++)
  {
  	_TxData[idx++] = highByte(value[i]);
  	_TxData[idx++] = lowByte(value[i]);
  }

  size = send_msg_pre(_TxData, idx);

	_function = MODBUS_FC_WRITE_AND_READ_REGISTERS;
	_length = size;
	_availableForWrite_flag = false;
	gpioPinWrite(_de_pin, _DEF_HIGH);
	uartWriteIT(_serial, (uint8_t *)_TxData, _length);

  return true;
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
