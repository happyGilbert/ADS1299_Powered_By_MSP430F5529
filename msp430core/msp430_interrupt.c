/*
 * msp430_interrupt.c
 *
 *  Created on: 2015Äê9ÔÂ4ÈÕ
 *      Author: jfanl
 */
#include <stdio.h>
#include <string.h>

#include "gpio.h"

#include "msp430_interrupt.h"

#define PINS_PER_PORT   (8)
struct msp430_int_s {
    void (*p1_cbs[PINS_PER_PORT])(void);
    void (*p2_cbs[PINS_PER_PORT])(void);
    unsigned char p1_exit[PINS_PER_PORT];
    unsigned char p2_exit[PINS_PER_PORT];
    /* Masks for each GPIO pin. */
    unsigned char p1_active_low;
    unsigned char p2_active_low;
};
static struct msp430_int_s msp_int = {0};

static inline unsigned char int_pin_to_index(unsigned short pin)
{
    /* Remove INT_PORT_Px from pin; index = pin/2 - 1. */
    return ((pin & 0x1F) >> 1) - 1;
}

/* Why does this even deserve its own function? */
static inline unsigned char index_to_pin(unsigned char index)
{
    return 1 << index;
}

void msp430_int_init(void)
{
    unsigned char index;

    GPIO_clearInterrupt(
    		GPIO_PORT_PA,
    		GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3 +
    		GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7 +
    		GPIO_PIN8 + GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
    		GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 + GPIO_PIN15
    		);
    GPIO_disableInterrupt(
    		GPIO_PORT_PA,
    		GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3 +
    		GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7 +
    		GPIO_PIN8 + GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
    		GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 + GPIO_PIN15
    		);

    for (index = 0; index < 8; index++) {
        unsigned char gpio_pin = index_to_pin(index);
        if (msp_int.p1_cbs[index]) {
        	GPIO_enableInterrupt(GPIO_PORT_P1, gpio_pin);
        }
        if (msp_int.p2_cbs[index]) {
        	GPIO_enableInterrupt(GPIO_PORT_P2, gpio_pin);
        }
    }
}

void msp430_int_disable(void)
{
	GPIO_clearInterrupt(
	    		GPIO_PORT_PA,
	    		GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3 +
	    		GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7 +
	    		GPIO_PIN8 + GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
	    		GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 + GPIO_PIN15
	    		);
	GPIO_disableInterrupt(
	    		GPIO_PORT_PA,
	    		GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3 +
	    		GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7 +
	    		GPIO_PIN8 + GPIO_PIN9 + GPIO_PIN10 + GPIO_PIN11 +
	    		GPIO_PIN12 + GPIO_PIN13 + GPIO_PIN14 + GPIO_PIN15
	    		);
}

int8_t msp430_reg_int_cb(void (*cb)(void), unsigned short pin,
    unsigned char lp_exit, unsigned char active_low)
{
    unsigned char index, gpio_pin;

    index = int_pin_to_index(pin);
    gpio_pin = index_to_pin(index);

    if (pin & INT_PORT_P1) {
        msp_int.p1_cbs[index] = cb;
        if (active_low) {
        	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, gpio_pin);
            /* Flag set on falling edge. */
        	GPIO_selectInterruptEdge(GPIO_PORT_P1, gpio_pin, GPIO_HIGH_TO_LOW_TRANSITION);
            msp_int.p1_active_low |= gpio_pin;
        } else {
        	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, gpio_pin);
        	GPIO_selectInterruptEdge(GPIO_PORT_P1, gpio_pin, GPIO_LOW_TO_HIGH_TRANSITION);
            msp_int.p1_active_low &= ~gpio_pin;
        }

        if (!cb) {
            /* No callback registered. Disable this interrupt if necessary. */
            /* Set to output mode. */
        	GPIO_setAsOutputPin(GPIO_PORT_P1, gpio_pin);
            /* Disable interrupt, clear flag. */
            GPIO_clearInterrupt(GPIO_PORT_P1, gpio_pin);
            GPIO_disableInterrupt(GPIO_PORT_P1, gpio_pin);
            msp_int.p1_exit[index] = INT_EXIT_NONE;
        } else {
            /* Enable interrupt, clear flag. */
        	GPIO_clearInterrupt(GPIO_PORT_P1, gpio_pin);
        	GPIO_enableInterrupt(GPIO_PORT_P1, gpio_pin);
            msp_int.p1_exit[index] = lp_exit;
        }
    } else if (pin & INT_PORT_P2) {
        msp_int.p2_cbs[index] = cb;
        if (active_low) {
            /* If interrupt enabled, use pullup resistor.
             * If interrupt disabled, output high.
             */
        	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, gpio_pin);
            /* Flag set on falling edge. */
        	GPIO_selectInterruptEdge(GPIO_PORT_P2, gpio_pin, GPIO_HIGH_TO_LOW_TRANSITION);
            msp_int.p2_active_low |= gpio_pin;
        } else {
        	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, gpio_pin);
        	GPIO_selectInterruptEdge(GPIO_PORT_P2, gpio_pin, GPIO_LOW_TO_HIGH_TRANSITION);
            msp_int.p2_active_low &= ~gpio_pin;
        }

        if (!cb) {
            /* No callback registered. Disable this interrupt if necessary. */
            /* Set to output mode. */
        	GPIO_setAsOutputPin(GPIO_PORT_P2, gpio_pin);
            /* Disable interrupt, clear flag. */
            GPIO_clearInterrupt(GPIO_PORT_P2, gpio_pin);
            GPIO_disableInterrupt(GPIO_PORT_P2, gpio_pin);
            msp_int.p2_exit[index] = INT_EXIT_NONE;
        } else {
            /* Enable interrupt, clear flag. */
        	GPIO_clearInterrupt(GPIO_PORT_P2, gpio_pin);
        	GPIO_enableInterrupt(GPIO_PORT_P2, gpio_pin);
            msp_int.p2_exit[index] = lp_exit;
        }
    }
    return 0;
}

#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void)
{
    unsigned char index, gpio_pin;
    index = int_pin_to_index(P1IV);
    gpio_pin = index_to_pin(index);

    if (msp_int.p1_cbs[index])
        msp_int.p1_cbs[index]();
    GPIO_clearInterrupt(GPIO_PORT_P1, gpio_pin);

    switch (msp_int.p1_exit[index]) {
    case INT_EXIT_NONE:
        break;
    case INT_EXIT_LPM0:
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case INT_EXIT_LPM1:
        __bic_SR_register_on_exit(LPM1_bits);
        break;
    case INT_EXIT_LPM2:
        __bic_SR_register_on_exit(LPM2_bits);
        break;
    case INT_EXIT_LPM3:
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case INT_EXIT_LPM4:
        __bic_SR_register_on_exit(LPM4_bits);
        break;
    default:
        break;
    }
}

#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void)
{
    unsigned char index, gpio_pin;
    index = int_pin_to_index(P2IV);
    gpio_pin = index_to_pin(index);

    if (msp_int.p2_cbs[index])
        msp_int.p2_cbs[index]();
    GPIO_clearInterrupt(GPIO_PORT_P2, gpio_pin);

    switch (msp_int.p2_exit[index]) {
    case INT_EXIT_NONE:
        break;
    case INT_EXIT_LPM0:
        __bic_SR_register_on_exit(LPM0_bits);
        break;
    case INT_EXIT_LPM1:
        __bic_SR_register_on_exit(LPM1_bits);
        break;
    case INT_EXIT_LPM2:
        __bic_SR_register_on_exit(LPM2_bits);
        break;
    case INT_EXIT_LPM3:
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case INT_EXIT_LPM4:
        __bic_SR_register_on_exit(LPM4_bits);
        break;
    default:
        break;
    }
}




