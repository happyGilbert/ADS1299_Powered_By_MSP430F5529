/*
 * msp430_uart.h
 *
 *  Created on: 2015Äê9ÔÂ4ÈÕ
 *      Author: jfanl
 */

#ifndef MSP430_UART_H_
#define MSP430_UART_H_

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
//! \brief Initializes the UART with the specified baud rate.
//!
//! Use USCI_A1 for UART communication.
//! Initializes the UART with the specified baud rate.
//!
//! \param baud is the baud rate of UART communication.
//
//*****************************************************************************
void msp430_uart_init(unsigned long baud);
//*****************************************************************************
//
//! \brief Get the number of bytes available for reading in the UART receive
//!      buffer.
//!
//!  The data received from UART will be stored in the UART receive buffer.
//!  This function get the number of data that stored in the UART receive
//!      buffer.
//!
//! \return the number of bytes available for reading
//
//*****************************************************************************
uint8_t msp430_uart_available();
//*****************************************************************************
//
//! \brief Get the next byte (character) of received data without removing it
//!      from the internal UART receive buffer.
//!
//! \param data is the next byte data get from internal UART receive buffer.
//!
//! \return 0 for succeed, -1 fail.
//
//*****************************************************************************
int8_t msp430_uart_peek(uint8_t *data);
//*****************************************************************************
//
//! \brief Get the next byte (character) of received data and remove it
//!      from the internal UART receive buffer.
//!
//! \param data is the next byte data get from internal UART receive buffer.
//!
//! \return 0 for succeed, -1 fail.
//
//*****************************************************************************
int8_t msp430_uart_read(uint8_t *data);
//*****************************************************************************
//
//! \brief Waits for All transmission of outgoing data to complete
//
//*****************************************************************************
void msp430_uart_flush();
//*****************************************************************************
//
//! \brief Transmit the specified length data via UART.
//!
//! The data will write into internal transmit buffer, then program will get
//!      data from transmit buffer and sent them to UART port.
//!
//! \param data point to the data that will be transmited.
//! \param data is the length of data that will be tranmited.
//!
//! \return None
//
//*****************************************************************************
void msp430_uart_write(uint8_t *data, uint8_t length);


#ifdef __cplusplus
}
#endif

#endif /* MSP430_UART_H_ */
