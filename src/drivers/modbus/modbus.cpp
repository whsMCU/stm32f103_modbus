/*
 * modbus.cpp
 *
 *  Created on: Feb 13, 2025
 *      Author: WANG
 */

#include <Modbus.h>

Modbus::Modbus(void)
{
  _idle = 0;
  _preTransmission = 0;
  _postTransmission = 0;
}


void Modbus::begin(uint8_t serial, uint8_t de_pin)
{
  _serial = serial;
  _de_pin = de_pin;
}

void Modbus::idle(void (*idle)())
{
  _idle = idle;
}


void Modbus::preTransmission(void (*preTransmission)())
{
  _preTransmission = preTransmission;
}


void Modbus::postTransmission(void (*postTransmission)())
{
  _postTransmission = postTransmission;
}

/* Reads the boolean status of bits and sets the array elements
   in the destination to TRUE or FALSE (single bits). */
int modbus_read_bits(int addr, int nb, uint8_t *dest)
{
    int rc;

    if (nb > MODBUS_MAX_READ_BITS) {

        return -1;
    }

    rc = read_io_status(ctx, MODBUS_FC_READ_COILS, addr, nb, dest);

    if (rc == -1)
        return -1;
    else
        return nb;
}


/* Same as modbus_read_bits but reads the remote device input table */
int modbus_read_input_bits(int addr, int nb, uint8_t *dest)
{
    int rc;


    if (nb > MODBUS_MAX_READ_BITS) {
        return -1;
    }

    rc = read_io_status(ctx, MODBUS_FC_READ_DISCRETE_INPUTS, addr, nb, dest);

    if (rc == -1)
        return -1;
    else
        return nb;
}

/* Reads the holding registers of remote device and put the data into an
   array */
int modbus_read_registers(int addr, int nb, uint16_t *dest)
{
    int status;

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        return -1;
    }

    status = read_registers(ctx, MODBUS_FC_READ_HOLDING_REGISTERS,
                            addr, nb, dest);
    return status;
}

/* Reads the input registers of remote device and put the data into an array */
int modbus_read_input_registers(int addr, int nb,
                                uint16_t *dest)
{
    int status;

    if (nb > MODBUS_MAX_READ_REGISTERS) {
        return -1;
    }

    status = read_registers(ctx, MODBUS_FC_READ_INPUT_REGISTERS,
                            addr, nb, dest);

    return status;
}

uint8_t Modbus::requestFrom(uint8_t slaveId, uint8_t type, uint8_t address, uint8_t nb)
{

  int valueSize = (type == COILS || type == DISCRETE_INPUTS) ? sizeof(uint8_t) : sizeof(uint16_t);

  _values = realloc(_values, nb * valueSize);

  if (_values == NULL) {
    //errno = ENOMEM;

    return 0;
  }

  int result = -1;

  if (slaveId >= 0 && slaveId <= 247)
  {
  	_slaveId = slaveId;
  }else
  {
  	return 0;
  }


  switch (type) {
    case COILS:
      result = modbus_read_bits(address, nb, (uint8_t*)_values);
      break;

    case DISCRETE_INPUTS:
      result = modbus_read_input_bits(address, nb, (uint8_t*)_values);
      break;

    case HOLDING_REGISTERS:
      result = modbus_read_registers(address, nb, (uint16_t*)_values);
      break;

    case INPUT_REGISTERS:
      result = modbus_read_input_registers(address, nb, (uint16_t*)_values);
      break;

    default:
      break;
  }

  if (result == -1) {
    return 0;
  }

  _transmissionBegun = false;
  _type = type;
  _available = nb;
  _read = 0;
  _availableForWrite = 0;
  _written = 0;

  return nb;
}
