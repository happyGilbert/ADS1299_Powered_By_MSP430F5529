/*
 * msp430_interrupt_parameters.h
 *
 *  Created on: 2015Äê9ÔÂ6ÈÕ
 *      Author: jfanl
 */

#ifndef MSP430_INTERRUPT_PARAMETERS_H_
#define MSP430_INTERRUPT_PARAMETERS_H_

struct int_param_s {
    void (*cb)(void);
    unsigned short pin;
    unsigned char lp_exit;
    unsigned char active_low;
};

#endif /* MSP430_INTERRUPT_PARAMETERS_H_ */
