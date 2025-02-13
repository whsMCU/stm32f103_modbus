/*
 * modbus.h
 *
 *  Created on: Feb 13, 2025
 *      Author: WANG
 */

#ifndef SRC_DRIVERS_MODBUS_MODBUS_H_
#define SRC_DRIVERS_MODBUS_MODBUS_H_

#include "hw.h"

/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_FC_REPORT_SLAVE_ID           0x11
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17

#define MODBUS_BROADCAST_ADDRESS    0

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 1 page 12)
 * Quantity of Coils to read (2 bytes): 1 to 2000 (0x7D0)
 * (chapter 6 section 11 page 29)
 * Quantity of Coils to write (2 bytes): 1 to 1968 (0x7B0)
 */
#define MODBUS_MAX_READ_BITS              2000
#define MODBUS_MAX_WRITE_BITS             1968

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 3 page 15)
 * Quantity of Registers to read (2 bytes): 1 to 125 (0x7D)
 * (chapter 6 section 12 page 31)
 * Quantity of Registers to write (2 bytes) 1 to 123 (0x7B)
 * (chapter 6 section 17 page 38)
 * Quantity of Registers to write in R/W registers (2 bytes) 1 to 121 (0x79)
 */
#define MODBUS_MAX_READ_REGISTERS          125
#define MODBUS_MAX_WRITE_REGISTERS         123
#define MODBUS_MAX_WR_WRITE_REGISTERS      121
#define MODBUS_MAX_WR_READ_REGISTERS       125

#define COILS             0
#define DISCRETE_INPUTS   1
#define HOLDING_REGISTERS 2
#define INPUT_REGISTERS   3

class Modbus
{
  public:
    Modbus();

    void begin(uint8_t serial, uint8_t de_pin);
    void idle(void (*)());
    void preTransmission(void (*)());
    void postTransmission(void (*)());

    uint8_t requestFrom(uint8_t id, uint8_t type, uint8_t address, uint8_t nb);



  private:
    uint8_t  _serial;                                             ///< serial port num
    uint8_t  _de_pin;

    unsigned long _timeout;
    int _defaultId;

    bool _transmissionBegun;
    int _id;
    int _slaveId;
    int _type;
    int _address;
    int _nb;

    void* _values;
    int _available;
    int _read;
    int _availableForWrite;
    int _written;


    // Modbus timeout [milliseconds]
    static const uint16_t ResponseTimeout          = 2000; ///< Modbus timeout [milliseconds]


    // idle callback function; gets called during idle time between TX and RX
    void (*_idle)();
    // preTransmission callback function; gets called before writing a Modbus message
    void (*_preTransmission)();
    // postTransmission callback function; gets called after a Modbus message has been sent
    void (*_postTransmission)();
};


#endif /* SRC_DRIVERS_MODBUS_MODBUS_H_ */
