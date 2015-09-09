/*
 * msp430_spi_a0.h
 *
 *  Created on: 2015Äê9ÔÂ5ÈÕ
 *      Author: jfanl
 */

#ifndef MSP430_SPI_A0_H_
#define MSP430_SPI_A0_H_
//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"{
#endif
//*****************************************************************************
//
//! \brief Initializes USCI_A0 with SPI mode.
//!
//! CLK: 1MHZ.
//! ClockPhase: USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT.
//! ClockPolarity: USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW
//! MSB_first
//!
//! The Seeting is specified to ADS1299 communication.
//
//*****************************************************************************
void msp430_spi_a0_init();
//*****************************************************************************
//
//! \brief transmit and receive one byte via SPI.
//!
//! \param transmitData is the data to be transmited.
//!
//! \return received data.
//
//*****************************************************************************
uint8_t msp430_spi_a0_transmitData(uint8_t transmitData);

#ifdef __cplusplus
}
#endif

#endif /* MSP430_SPI_A0_H_ */
