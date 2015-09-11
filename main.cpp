/*
 * main.cpp
 *
 *  Created on: 2015Äê9ÔÂ9ÈÕ
 *      Author: jfanl
 */
#include "driverlib.h"
#include "msp430_clock.h"
#include "msp430_uart.h"
#include "ADS1299Manager.h"
#include "ads1299app.h"

static inline void msp430PlatformInit(void)
{
    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    //Set VCore = 1 for 12MHz clock
    PMM_setVCore(PMM_CORE_LEVEL_1);
    msp430_clock_init(12000000);
    msp430_uart_init(230400);
    ads1299_init(ADS_LEADS_16);
    /* Enable interrupts. */
	__bis_SR_register(GIE);
}

void main()
{
	void serialEvent();
	msp430PlatformInit();
	serialEvent();
//	ads1299_activateAllChannelsToTestCondition_fast_2X();
	ads1299_startRunning();
	while(1)
	{
		ads1299_update();
	    __bis_SR_register(LPM0_bits + GIE);
	}
}


void serialEvent()
{
	bool is_running = false;
	while(!is_running)
	{
		if(msp430_uart_available() > 0)
		{
			uint8_t data;
			msp430_uart_read(&data);
			   switch (data)
			    {
			      case 'b':
			    	  is_running = true;
			    	  break;
			     case '?':
			        //print state of all registers
			    	 ads1299_printAllchipRegister();
			        break;
			      default:
			        break;
			    }
		}
	}
}
