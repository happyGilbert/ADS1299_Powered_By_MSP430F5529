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

ADS1299Manager ADSManager;
bool is_running = false;
unsigned int sampleCounter = 0;
uint8_t gainCode = ADS_GAIN24;   //how much gain do I want
uint8_t inputType = ADSINPUT_NORMAL;   //here's the normal way to setup the channels

bool ads_rx_new = false;
static void ads_data_ready_cb()
{
	ads_rx_new = true;
}

static inline void msp430PlatformInit(void)
{
    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    //Set VCore = 1 for 12MHz clock
    PMM_setVCore(PMM_CORE_LEVEL_1);
    msp430_clock_init(12000000);
    msp430_uart_init(115200);
    /* Enable interrupts. */
	__bis_SR_register(GIE);
}

volatile uint8_t data0 = 0;

void main()
{
	void serialEvent();
	void startRunning();
	msp430PlatformInit();
	ADSManager.initialize(ads_data_ready_cb);
	for (int chan=1; chan <= 8; chan++) {
		ADSManager.activateChannel(chan, gainCode, inputType);
	}
	ADSManager.configureLeadOffDetection(LOFF_MAG_6NA, LOFF_FREQ_DC);
	for (int chan=1; chan <= 8; chan++) {
		ADSManager.changeChannelLeadOffDetection(chan, ON, NCHAN);
	}
//	ADSManager.changeChannelLeadOffDetection(1, ON, PCHAN);
	ADSManager.printAllRegisters();
	ads_rx_new = false;
	data0 = 0;
	data0 = ADSManager.getDeviceID();
	serialEvent();
	startRunning();
	while(1)
	{
	    if(ads_rx_new){
		    ADSManager.updateChannelData();
		    sampleCounter++;
		    ADSManager.writeChannelDataAsUAISLab(sampleCounter);
		    ads_rx_new = false;
	    }
	    __bis_SR_register(LPM0_bits + GIE);

	}
}

void stopRunning(void) {
  ADSManager.stop();                    // stop the data acquisition
  is_running = false;
}

void startRunning() {
    ADSManager.start();    //start the data acquisition
    is_running = true;
}

void changeChannelState_maintainRunningState(int chan, int start)
{
  bool is_running_when_called = is_running;

  //must stop running to change channel settings
  stopRunning();
  if (start == true) {
    ADSManager.activateChannel(chan,gainCode,inputType);
  } else {;
    ADSManager.deactivateChannel(chan);
  }

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning();
  }
}

void changeChannelLeadOffDetection_maintainRunningState(int chan, int start, int code_P_N_Both)
{
  bool is_running_when_called = is_running;

  //must stop running to change channel settings
  stopRunning();
  if (start == true) {
    ADSManager.changeChannelLeadOffDetection(chan,ON,code_P_N_Both);
  } else {
    ADSManager.changeChannelLeadOffDetection(chan,OFF,code_P_N_Both);
  }

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning();
  }
}


void activateAllChannelsToTestCondition(int testInputCode, uint8_t amplitudeCode, uint8_t freqCode)
{
  bool is_running_when_called = is_running;

  //set the test signal to the desired state
  ADSManager.configureInternalTestSignal(amplitudeCode,freqCode);

  //must stop running to change channel settings
  stopRunning();

  //loop over all channels to change their state
  for (int Ichan=1; Ichan <= 8; Ichan++) {
    ADSManager.activateChannel(Ichan,gainCode,testInputCode);  //Ichan must be [1 8]...it does not start counting from zero
  }

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning();
  }
}

#define ACTIVATE (1)
#define DEACTIVATE (0)
void serialEvent()
{
	while(!is_running)
	{
		if(msp430_uart_available() > 0)
		{
			uint8_t data;
			msp430_uart_read(&data);
			   switch (data)
			    {
			      //turn channels on and off
			      case '1':
			        changeChannelState_maintainRunningState(1,DEACTIVATE); break;
			      case '2':
			        changeChannelState_maintainRunningState(2,DEACTIVATE); break;
			      case '3':
			        changeChannelState_maintainRunningState(3,DEACTIVATE); break;
			      case '4':
			        changeChannelState_maintainRunningState(4,DEACTIVATE); break;
			      case '5':
			        changeChannelState_maintainRunningState(5,DEACTIVATE); break;
			      case '6':
			        changeChannelState_maintainRunningState(6,DEACTIVATE); break;
			      case '7':
			        changeChannelState_maintainRunningState(7,DEACTIVATE); break;
			      case '8':
			        changeChannelState_maintainRunningState(8,DEACTIVATE); break;
			      case 'q':
			        changeChannelState_maintainRunningState(1,ACTIVATE); break;
			      case 'w':
			        changeChannelState_maintainRunningState(2,ACTIVATE); break;
			      case 'e':
			        changeChannelState_maintainRunningState(3,ACTIVATE); break;
			      case 'r':
			        changeChannelState_maintainRunningState(4,ACTIVATE); break;
			      case 't':
			        changeChannelState_maintainRunningState(5,ACTIVATE); break;
			      case 'y':
			        changeChannelState_maintainRunningState(6,ACTIVATE); break;
			      case 'u':
			        changeChannelState_maintainRunningState(7,ACTIVATE); break;
			      case 'i':
			        changeChannelState_maintainRunningState(8,ACTIVATE); break;

			      //turn lead-off detection on and off
			      case '!':
			        changeChannelLeadOffDetection_maintainRunningState(1,ACTIVATE,PCHAN); break;
			      case '@':
			        changeChannelLeadOffDetection_maintainRunningState(2,ACTIVATE,PCHAN); break;
			      case '#':
			        changeChannelLeadOffDetection_maintainRunningState(3,ACTIVATE,PCHAN); break;
			      case '$':
			        changeChannelLeadOffDetection_maintainRunningState(4,ACTIVATE,PCHAN); break;
			      case '%':
			        changeChannelLeadOffDetection_maintainRunningState(5,ACTIVATE,PCHAN); break;
			      case '^':
			        changeChannelLeadOffDetection_maintainRunningState(6,ACTIVATE,PCHAN); break;
			      case '&':
			        changeChannelLeadOffDetection_maintainRunningState(7,ACTIVATE,PCHAN); break;
			      case '*':
			        changeChannelLeadOffDetection_maintainRunningState(8,ACTIVATE,PCHAN); break;
			      case 'Q':
			        changeChannelLeadOffDetection_maintainRunningState(1,DEACTIVATE,PCHAN); break;
			      case 'W':
			        changeChannelLeadOffDetection_maintainRunningState(2,DEACTIVATE,PCHAN); break;
			      case 'E':
			        changeChannelLeadOffDetection_maintainRunningState(3,DEACTIVATE,PCHAN); break;
			      case 'R':
			        changeChannelLeadOffDetection_maintainRunningState(4,DEACTIVATE,PCHAN); break;
			      case 'T':
			        changeChannelLeadOffDetection_maintainRunningState(5,DEACTIVATE,PCHAN); break;
			      case 'Y':
			        changeChannelLeadOffDetection_maintainRunningState(6,DEACTIVATE,PCHAN); break;
			      case 'U':
			        changeChannelLeadOffDetection_maintainRunningState(7,DEACTIVATE,PCHAN); break;
			      case 'I':
			        changeChannelLeadOffDetection_maintainRunningState(8,DEACTIVATE,PCHAN); break;
			       case 'A':
			        changeChannelLeadOffDetection_maintainRunningState(1,ACTIVATE,NCHAN); break;
			      case 'S':
			        changeChannelLeadOffDetection_maintainRunningState(2,ACTIVATE,NCHAN); break;
			      case 'D':
			        changeChannelLeadOffDetection_maintainRunningState(3,ACTIVATE,NCHAN); break;
			      case 'F':
			        changeChannelLeadOffDetection_maintainRunningState(4,ACTIVATE,NCHAN); break;
			      case 'G':
			        changeChannelLeadOffDetection_maintainRunningState(5,ACTIVATE,NCHAN); break;
			      case 'H':
			        changeChannelLeadOffDetection_maintainRunningState(6,ACTIVATE,NCHAN); break;
			      case 'J':
			        changeChannelLeadOffDetection_maintainRunningState(7,ACTIVATE,NCHAN); break;
			      case 'K':
			        changeChannelLeadOffDetection_maintainRunningState(8,ACTIVATE,NCHAN); break;
			      case 'Z':
			        changeChannelLeadOffDetection_maintainRunningState(1,DEACTIVATE,NCHAN); break;
			      case 'X':
			        changeChannelLeadOffDetection_maintainRunningState(2,DEACTIVATE,NCHAN); break;
			      case 'C':
			        changeChannelLeadOffDetection_maintainRunningState(3,DEACTIVATE,NCHAN); break;
			      case 'V':
			        changeChannelLeadOffDetection_maintainRunningState(4,DEACTIVATE,NCHAN); break;
			      case 'B':
			        changeChannelLeadOffDetection_maintainRunningState(5,DEACTIVATE,NCHAN); break;
			      case 'N':
			        changeChannelLeadOffDetection_maintainRunningState(6,DEACTIVATE,NCHAN); break;
			      case 'M':
			        changeChannelLeadOffDetection_maintainRunningState(7,DEACTIVATE,NCHAN); break;
			      case '<':
			        changeChannelLeadOffDetection_maintainRunningState(8,DEACTIVATE,NCHAN); break;

			      //control the bias generation
			      case '`':
			        ADSManager.setAutoBiasGeneration(true); break;
			      case '~':
			        ADSManager.setAutoBiasGeneration(false); break;

			      //control test signals
			      case '0':
			        activateAllChannelsToTestCondition(ADSINPUT_SHORTED,ADSTESTSIG_NOCHANGE,ADSTESTSIG_NOCHANGE); break;
			      case '-':
			        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_SLOW); break;
			      case '+':
			        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); break;
			      case '=':
			        //repeat the line above...just for human convenience
			        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); break;
			      case 'p':
			        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_DCSIG); break;
			      case '[':
			        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_SLOW); break;
			      case ']':
			        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_FAST); break;

			      //other commands
			      case 'b':
			    	  is_running = true;
			    	  break;
			     case '?':
			        //print state of all registers
			        ADSManager.printAllRegisters();
			        break;
			      default:
			        break;
			    }
		}
	}
}
