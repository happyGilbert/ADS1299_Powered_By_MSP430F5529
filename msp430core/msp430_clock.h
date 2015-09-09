/*
 * msp430_clock.h
 *
 *  Created on: 2015Äê9ÔÂ3ÈÕ
 *      Author: jfanl
 */

#ifndef MSP430_CLOCK_H_
#define MSP430_CLOCK_H_
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

#define CPU_F ((double)12000000)
#define msp430_delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0))
//*****************************************************************************
//
//! \brief Initializes the system clock and delay and get running times function.
//!
//! Initializes the MCLK and SMCLK source to the  DCO.
//! Initializes the ACLK source to the  XT1: 32.768K.
//! Initializes the Timer B0 for delay_ms()function ande get_clock_ms()
//!        function!
//!
//! \param _mclk_khz is the target frequency for MCLK and SMCLK in kHz
//!
//! \return None
//
//*****************************************************************************
void msp430_clock_init(uint32_t _mclk);
//*****************************************************************************
//
//! \brief Get the current MCLK frequency
//!
//! Get the current MCLK frequency.
//!
//! \return Current MCLK frequency in Hz
//
//*****************************************************************************
uint32_t msp430_get_mclk_freq();
//*****************************************************************************
//
//! \brief Get the current SMCLK frequency
//!
//! Get the current SMCLK frequency.
//!
//! \return Current SMCLK frequency in Hz
//
//*****************************************************************************
uint32_t msp430_get_smclk_freq();
//*****************************************************************************
//
//! \brief Get the current ACLK frequency
//!
//! Get the current ACLK frequency.
//!
//! \return Current ACLK frequency in Hz
//
//*****************************************************************************
uint32_t msp430_get_aclk_freq();
//*****************************************************************************
//
//! \brief Get the running times of system.
//!
//! \param times is the running times of system in ms.
//!
//! \return None.
//
//*****************************************************************************
void msp430_get_clock_ms(unsigned long *times);
//*****************************************************************************
//
//! \brief Make delay for program.
//!
//! Make delay for program.
//!
//! \param num_ms is the delay times.
//!
//! \return None.
//
//*****************************************************************************
void msp430_delay_ms(unsigned long num_ms);
//*****************************************************************************
//
//! \brief Register a time based event function.
//!
//! Register a time event function.
//!
//! \param timer_cb point to the event function.
//! \param num_ms is the remaining Time for function will be executed.
//!
//! \return None.
//
//*****************************************************************************
void msp430_register_timer_cb(void (*timer_cb)(void), unsigned long num_ms);
//*****************************************************************************
//
//! \brief Output MCLK in P7.7.
//!
//! Output MCLK in P7.7.
//!
//! \return None.
//
//*****************************************************************************
void msp430_mclk_output_enable();
//*****************************************************************************
//
//! \brief Output SMCLK in P2.2.
//!
//! Output MCLK in P2.2.
//!
//! \return None.
//
//*****************************************************************************
void msp430_smclk_output_enable();
//*****************************************************************************
//
//! \brief Output MCLK in P1.0.
//!
//! Output MCLK in P1.0.
//!
//! \return None.
//
//*****************************************************************************
void msp430_aclk_output_enable();

#ifdef __cplusplus
}
#endif

#endif /* MSP430_CLOCK_H_ */
