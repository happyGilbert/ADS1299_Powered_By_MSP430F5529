/*
 * msp430_uart.c
 *
 *  Created on: 2015Äê9ÔÂ4ÈÕ
 *      Author: jfanl
 */

#include "usci_a_uart.h"
#include "msp430_clock.h"
#include "msp430_uart.h"
#include "gpio.h"
#include "msp430.h"

#define UART_BUFFER_SIZE 16
struct ringBuf{
	unsigned char buf[UART_BUFFER_SIZE];
	volatile uint8_t head;
	volatile uint8_t tail;
};

struct ringBuf _rxBuf = {{0}, 0, 0};
struct ringBuf _txBuf = {{0}, 0, 0};

static inline void store_char(unsigned char c, struct ringBuf *buffer)
{
	uint8_t i = (unsigned int)(buffer->head + 1) % UART_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != buffer->tail) {
		(buffer->buf[buffer->head]) = c;
		buffer->head = i;
	}
}


void msp430_uart_init(unsigned long baud)
{
	unsigned long divider;
	unsigned long _smclk;

	_smclk = msp430_get_smclk_freq();

    /**Configure UART pins
     * Set P4.4 and P4.5 as Secondary Module Function Input.
     * Select Port 4
     * Set Pin 4, 5 to input Secondary Module Function, (UCA1TXD/UCA1SIMO, UCA1RXD/UCA1SOMI).
     **/
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P4,
        GPIO_PIN4 + GPIO_PIN5
        );

    divider=(_smclk<<3)/baud;   //fixed-point number: q3

    // Configure UART
    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = divider>>3;
    param.firstModReg = 0;
    param.secondModReg = (divider & 0x07);
    param.parity = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    USCI_A_UART_init(USCI_A1_BASE, &param);
    USCI_A_UART_enable(USCI_A1_BASE);
    USCI_A_UART_clearInterrupt(USCI_A1_BASE,
    		                   USCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    // Enable USCI_A1 RX interrupt
    USCI_A_UART_enableInterrupt(USCI_A1_BASE,
    		                    USCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt
}

uint8_t msp430_uart_available()
{
	return (uint8_t)(UART_BUFFER_SIZE + _rxBuf.head - _rxBuf.tail) % UART_BUFFER_SIZE;
}

int8_t msp430_uart_peek(uint8_t *data)
{
	if (_rxBuf.head == _rxBuf.tail) {
		return -1;
	} else {
		*data = _rxBuf.buf[_rxBuf.tail];
		return 0;
	}
}

int8_t msp430_uart_read(uint8_t *data)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (_rxBuf.head == _rxBuf.tail) {
		return -1;
	} else {
		*data = _rxBuf.buf[_rxBuf.tail];
		_rxBuf.tail = (unsigned int)(_rxBuf.tail + 1) % UART_BUFFER_SIZE;
		return 0;
	}
}

void msp430_uart_flush()
{
	while (_txBuf.head != _txBuf.tail);
}

void msp430_uart_write(uint8_t *data, uint8_t length)
{
	uint8_t i = (_txBuf.head + 1) % UART_BUFFER_SIZE;
	uint8_t len;

	// If the output buffer is full, there's nothing for it other than to
	// wait for the interrupt handler to empty it a bit
	// ???: return 0 here instead?
	while (i == _txBuf.tail);

	_txBuf.buf[_txBuf.head] = *data;
	_txBuf.head = i;
    // Enable USCI_A1 TX interrupt
	USCI_A_UART_enableInterrupt(USCI_A1_BASE,
    		                    USCI_A_UART_TRANSMIT_INTERRUPT); // Enable interrupt
	for(len = 0; len <length - 1; len++)
	{
		data++;
		i = (_txBuf.head + 1) % UART_BUFFER_SIZE;
		while (i == _txBuf.tail);
		{
		    // Enable USCI_A1 TX interrupt
		    USCI_A_UART_enableInterrupt(USCI_A1_BASE,
		    		                    USCI_A_UART_TRANSMIT_INTERRUPT); // Enable interrupt
		}
		_txBuf.buf[_txBuf.head] = *data;
		_txBuf.head = i;
	}
}

static inline void uart_rx_isr(void)
{
	unsigned char c = USCI_A_UART_receiveData(USCI_A1_BASE);
	store_char(c, &_rxBuf);
}

static inline void uart_tx_isr(void)
{
	if (_txBuf.head == _txBuf.tail) {     //No data need to transmit.
		USCI_A_UART_disableInterrupt(USCI_A1_BASE,
                                     USCI_A_UART_TRANSMIT_INTERRUPT); // Disable interrupt
		UCA1IFG |= UCTXIFG;    // Set Flag again
		return;
	}

	unsigned char data = _txBuf.buf[_txBuf.tail];
	_txBuf.tail = (_txBuf.tail + 1) % UART_BUFFER_SIZE;
	USCI_A_UART_transmitData(USCI_A1_BASE, data);
}

//******************************************************************************
//
//This is the USCI_A1 interrupt vector service routine.
//
//******************************************************************************
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
	switch(__even_in_range(UCA1IV,4))
    {
    case USCI_NONE: break;
    //Vector 2 - RXIFG
    case 2:
    	uart_rx_isr();
        break;
    //Vector 4 - TXIFG
    case 4:
    	uart_tx_isr();
    	break;
    default: break;
    }
}

