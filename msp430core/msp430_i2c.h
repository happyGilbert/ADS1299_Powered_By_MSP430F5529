/*
 * msp430_i2c.h
 *
 *  Created on: 2015Äê9ÔÂ4ÈÕ
 *      Author: jfanl
 */

#ifndef MSP430_I2C_H_
#define MSP430_I2C_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
//*****************************************************************************
//
//! \brief Set up the I2C port and configure the MSP430 as the master.
//
//*****************************************************************************
void msp430_i2c_init(void);
//*****************************************************************************
//
//! \brief Disable I2C communication.
//!
//! This function will disable the I2C hardware and should be called prior to
//!  entering low-power mode.
//
//*****************************************************************************
void msp430_i2c_disable(void);
//*****************************************************************************
//
//! \brief Write to a device register.
//!
//! \param slave_addr is the slave address of device.
//! \param reg_addr   is slave register to be written to.
//! \param length     is the number of bytes to write.
//! \param data       is data to be written to register.
//!
//! \return 0 succeed, -1 fail.
//
//*****************************************************************************
int8_t msp430_i2c_write(unsigned char slave_addr,
        unsigned char reg_addr,
        unsigned char length,
        unsigned char const *data);
//*****************************************************************************
//
//! \brief Write to a device register.
//!
//! \param slave_addr is the slave address of device.
//! \param reg_addr   is slave register to be read from.
//! \param length     is the number of bytes to read.
//! \param data       is data from register.
//!
//! \return 0 succeed, -1 fail.
//
//*****************************************************************************
int8_t msp430_i2c_read(unsigned char slave_addr,
        unsigned char reg_addr,
        unsigned char length,
        unsigned char *data);

#ifdef __cplusplus
}
#endif

#endif /* MSP430_I2C_H_ */
