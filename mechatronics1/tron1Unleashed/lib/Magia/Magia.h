//HHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Magia.h HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
// Filename:    Magia.h
// Description: Magic library that does some complex things.
// Author:      Danon Bradford
// Date:		2019-04-24
//HHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Magia.h HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH

#ifndef DANON_MAGIA_H_
#define DANON_MAGIA_H_

#ifdef __cplusplus
extern "C" {
#endif

//IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
// Header Files
//-----------------------------------------------------------------------------
#include <stdint.h>

//=============================================================================
// Public Enumerated Constants Definitions
//-----------------------------------------------------------------------------
// 7-bit addresses shifted left by 1. 
// These 8-bit enumerations include the R/W bit.
typedef enum
{
    PCF8574 = 0b01001110, 
    PCF8574A = 0b01111110,
} SlaveAddresses;

//=============================================================================
// Public Function Declarations
//-----------------------------------------------------------------------------

//=============================================================================
// Magia_Init
//
// Initialise the "magic under the hood".
// Input:
//    None.
// Output:
//    None.
// Conditions:
//    Hardware has been correctly wired.
//-----------------------------------------------------------------------------
void Magia_Init(void);

//=============================================================================
// Magia_WriteByte
//
// Write a single 8-bit byte of data to a slave device on the I2C bus.
// Input:
//    slaveAddr    - The slave address of the I2C device.
//    writeByte    - Byte to be written.
// Output:
//    uint16_t     - 0x1001 = I2C bus is not free.
//    uint16_t     - 0x2002 = Slave is not there.
//    uint16_t     - 0x3004 = Slave did not ack all data.
//    uint16_t     - 0x3000 = no error.
// Conditions:
//    Magia_Init() has been called. 
//-----------------------------------------------------------------------------
uint16_t Magia_WriteByte(const uint8_t slaveAddr, const uint8_t writeByte);

//=============================================================================
// Magia_WriteBytes
//
// Write an 8-bit buffer of data to a slave device on the I2C bus.
// Input:
//    slaveAddr    - The slave address of the I2C device.
//    writeBuffPtr - Pointing to the address of the data buffer to be written.
//    writeLength  - The number of bytes to write.
// Output:
//    uint16_t     - 0x1001 = I2C bus is not free.
//    uint16_t     - 0x2002 = Slave is not there.
//    uint16_t     - 0x3004 = Slave did not ack all data.
//    uint16_t     - 0x3000 = no error.
// Conditions:
//    Magia_Init() has been called. 
//-----------------------------------------------------------------------------
uint16_t Magia_WriteBytes(const uint8_t slaveAddr, uint8_t *const writeBuffPtr, uint8_t writeLength);

#ifdef  __cplusplus
}
#endif

#endif /* DANON_MAGIA_H_ */

// Magia.h EOF