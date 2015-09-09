/*
 * msp430_spi_a0.c
 *
 *  Created on: 2015Äê9ÔÂ5ÈÕ
 *      Author: jfanl
 */
#include "ucs.h"
#include "gpio.h"
#include "usci_a_spi.h"
#include "msp430_spi_a0.h"

uint8_t _wasReceived = 0;

void msp430_spi_a0_init()
{
    /**Configure Pins for SPI
     * Set P2.7, P3.3 and P3.4 as Secondary Module Function.
     * */
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P2,
            GPIO_PIN7
            );
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P3,
            GPIO_PIN3 + GPIO_PIN4
            );
    //Initialize Master
    USCI_A_SPI_initMasterParam param = {0};
    param.selectClockSource = USCI_A_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = UCS_getSMCLK();
    param.desiredSpiClock = 4000000;
    param.msbFirst = USCI_A_SPI_MSB_FIRST;
    param.clockPhase = USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    USCI_A_SPI_initMaster(USCI_A0_BASE, &param);

    //Enable SPI module
    USCI_A_SPI_enable(USCI_A0_BASE);

    //Enable Receive interrupt
    USCI_A_SPI_clearInterrupt(USCI_A0_BASE,
                              USCI_A_SPI_RECEIVE_INTERRUPT);
    USCI_A_SPI_enableInterrupt(USCI_A0_BASE,
                               USCI_A_SPI_RECEIVE_INTERRUPT);
}

uint8_t msp430_spi_a0_transmitData(uint8_t transmitData)
{
    //USCI_A0 TX buffer ready?
    while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE,
    		USCI_A_SPI_TRANSMIT_INTERRUPT));
    USCI_A_SPI_transmitData(USCI_A0_BASE, transmitData);
    //USCI_A0 TX buffer ready?
    while(!_wasReceived)
    {
    	//Enter LPM0 mode, wait receive interrupt.
    	__bis_SR_register(LPM0_bits + GIE);
    }
    _wasReceived = 0;
//    while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE,
//    		USCI_A_SPI_RECEIVE_INTERRUPT));
    return USCI_A_SPI_receiveData(USCI_A0_BASE);
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,4))
    {
    //Vector 2 - RXIFG
    case 2:
    	_wasReceived = 1;
    	__bic_SR_register_on_exit(LPM0_bits);
        break;
    default: break;
    }
}
