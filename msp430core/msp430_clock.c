/*
 * msp430_clock.c
 *
 *  Created on: 2015Äê9ÔÂ3ÈÕ
 *      Author: jfanl
 */
#include "stdio.h"

#include "ucs.h"
#include "gpio.h"
#include "timer_b.h"

#include "msp430_clock.h"

#define REF_FREQUENCE  32768  //DCO reference

struct msp430_clock_s {
    volatile uint32_t timestamp;       //running times of system.
    unsigned short ms_per_interrupt;   //timer b0 interrupt time interval.
    unsigned short ticks_per_interrupt;//timer b0 count to getting time interval.
    unsigned long timer_remaining_ms;  //remaining Time for time based event function will be executed.
    void (*timer_cb)(void);            //point to function for register event.
};
static struct msp430_clock_s clock = {
    .timer_remaining_ms = 0,
    .timer_cb = NULL
};


/**XT1: 32.768k
 * XT2: unused
 * MCLK: DCO
 * SMCLK: DCO
 * ACLK: XT1
 * */
void msp430_clock_init(uint32_t _mclk)
{
	//Port select XT1
	GPIO_setAsPeripheralModuleFunctionOutputPin(
		GPIO_PORT_P5,
		GPIO_PIN4
		);

	UCS_setExternalClockSource(32768, 0);    //XT1: 32.768k, XT2: unused.
	UCS_turnOffXT2();
	//Initializes the XT1 crystal oscillator with no timeout
	//In case of failure, code hangs here.
	//For time-out instead of code hang use UCS_turnOnLFXT1WithTimeout()
	UCS_turnOnLFXT1(UCS_XT1_DRIVE_0,
	                UCS_XCAP_3
	                );

	UCS_enableClockRequest(UCS_ACLK + UCS_SMCLK + UCS_MCLK);  //enable clock request for mclk, smclk, aclk.

    //Set DCO FLL reference = REFO
    UCS_initClockSignal(UCS_FLLREF,
                        UCS_REFOCLK_SELECT,
                        UCS_CLOCK_DIVIDER_1
                        );
    //Set ACLK = XT1: 32.768k
    UCS_initClockSignal(UCS_ACLK,
    		            UCS_XT1CLK_SELECT,
                        UCS_CLOCK_DIVIDER_1
                        );

    //initialize DCO with no timeout.
	//In case of failure, code hangs here.
	//For time-out instead of code hang use UCS_initFLLSettle().
    //Set MCLK and SMCLK frequence to MCLK_FREQUENCE_KHZ, sourced from DCO.
    UCS_initFLL(_mclk / 1000,
    		    (uint16_t)(_mclk / REF_FREQUENCE)
        );

    UCS_turnOnSMCLK();

//	msp430_mclk_output_enable();
//	msp430_smclk_output_enable();
//	msp430_aclk_output_enable();


    clock.ticks_per_interrupt = UCS_getSMCLK() / 40 / 1000;
	clock.ms_per_interrupt = 1;
	clock.timestamp = 0;
	clock.timer_cb = NULL;
	clock.timer_remaining_ms = 0;

	//enable timer b0 contiuous mode.
    Timer_B_initContinuousModeParam initContParam = {0};
    initContParam.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_40;
    initContParam.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
    initContParam.timerClear = TIMER_B_DO_CLEAR;
    initContParam.startTimer = false;
    Timer_B_initContinuousMode(TIMER_B0_BASE, &initContParam);

    //Initiaze compare mode to getting interrupt at every 1ms passed.
    Timer_B_clearCaptureCompareInterrupt(TIMER_B0_BASE,
                                         TIMER_B_CAPTURECOMPARE_REGISTER_0);

    Timer_B_initCompareModeParam initCompParam = {0};
    initCompParam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_0;
    initCompParam.compareInterruptEnable =
        TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
    initCompParam.compareOutputMode = TIMER_B_OUTPUTMODE_OUTBITVALUE;
    initCompParam.compareValue = clock.ticks_per_interrupt;
    Timer_B_initCompareMode(TIMER_B0_BASE, &initCompParam);

    Timer_B_startCounter(TIMER_B0_BASE,
                         TIMER_B_CONTINUOUS_MODE
                         );
}

#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR (void)
{
    uint16_t compVal = Timer_B_getCaptureCompareCount(TIMER_B0_BASE,
                                                      TIMER_B_CAPTURECOMPARE_REGISTER_0)
                       + clock.ticks_per_interrupt;
    // Add Offset to CCR0 [Cont mode]
    Timer_B_setCompareValue(TIMER_B0_BASE,
                            TIMER_B_CAPTURECOMPARE_REGISTER_0,
                            compVal
                            );
    clock.timestamp += clock.ms_per_interrupt;
    if (clock.timer_remaining_ms) {
        clock.timer_remaining_ms -= clock.ms_per_interrupt;
        if (!clock.timer_remaining_ms)
            clock.timer_cb();
    }

    __bic_SR_register_on_exit(LPM0_bits);
}

uint32_t msp430_get_mclk_freq()
{
    return UCS_getSMCLK();
}

uint32_t msp430_get_smclk_freq()
{
    return UCS_getSMCLK();
}

uint32_t msp430_get_aclk_freq()
{
    return UCS_getACLK();
}

void msp430_get_clock_ms(unsigned long *times)
{
    if (!times)
        return;
    times[0] = clock.timestamp;
}

void msp430_delay_ms(unsigned long num_ms)
{
    uint32_t start_time = clock.timestamp;
    while (clock.timestamp - start_time < num_ms)
        __bis_SR_register(LPM0_bits + GIE);
}

void msp430_register_timer_cb(void (*timer_cb)(void), unsigned long num_ms)
{
    if (!timer_cb || !num_ms) {
        clock.timer_cb = NULL;
        clock.timer_remaining_ms = 0;
        return;
    }

    /* Timer count needs to be evenly divisible by clock.ms_per_interrupt to
     * avoid overflow.
     */
    clock.timer_remaining_ms = num_ms + (clock.timer_remaining_ms %
        clock.ms_per_interrupt);
    clock.timer_cb = timer_cb;
    return;
}

void msp430_mclk_output_enable()
{
	GPIO_setAsPeripheralModuleFunctionOutputPin(
		GPIO_PORT_P7,
		GPIO_PIN7
		);
}

void msp430_smclk_output_enable()
{
	GPIO_setAsPeripheralModuleFunctionOutputPin(
		GPIO_PORT_P2,
		GPIO_PIN2
		);
}

void msp430_aclk_output_enable()
{
	GPIO_setAsPeripheralModuleFunctionOutputPin(
		GPIO_PORT_P1,
		GPIO_PIN0
		);
}

