/*
 * ads1299app.cpp
 *
 *  Created on: 2015Äê9ÔÂ10ÈÕ
 *      Author: jfanl
 */
#include "ads1299app.h"

ADS1299Manager ADSManager;
unsigned long sampleCounter = 0; //record sampling number
static bool ads_rx_new = false;  //flag for new data ready

//********************************************************************************************
//! \brief DRDY pin interrupt function.
//
//********************************************************************************************
static void ads_data_ready_cb()
{
	ads_rx_new = true;         //true means new data ready.
}

//volatile uint8_t data0 = 0;

//********************************************************************************************
//! \brief Initializes ADS1299 app layer.
//!
//! \param[in] ads_leads is the leads of multi-chip ads1299: ADS_LEADS_8,
//!             ADS_LEADS_16, ADS_LEADS_24(Reserve) or ADS_LEADS_32(Reserve).
//
//********************************************************************************************
void ads1299_init(uint8_t ads_leads){
	ADSManager.initialize(ads_data_ready_cb,ads_leads);
	ads1299_activeAllChannelToNormalOperation();
	ads1299_activeLeadOFFDetection();
	ads1299_printAllchipRegister();               //print register information. if not use, comment it.
	ads_rx_new = false;
//	data0 = 0;
//	data0 = ADSManager.getDeviceID(ADS_CHIP_ONE);
}

//********************************************************************************************
//! \brief Configure all channel to normal data continuous sampling mode.
//
//********************************************************************************************
void ads1299_activeAllChannelToNormalOperation(){
	for(uint8_t chips = 0; chips < ADSManager.ADS_LEADS; chips++){
		for (uint8_t chan=1; chan <= 8; chan++) {
			ADSManager.activateChannel(chan, GAIN, ADSINPUT_NORMAL,chips);
		}
	}
}

//********************************************************************************************
//! \brief Active channel lead-off detection.
//! \for negative input use n-channel lead-off detection.
//! \for positive input use p-channel lead-off detection.]
//! \not available in internal testing siganl mode.
//
//********************************************************************************************
void ads1299_activeLeadOFFDetection(){
	ADSManager.configureLeadOffDetection(LOFF_MAG_6NA, LOFF_FREQ_DC);
	for(uint8_t chips = 0; chips < ADSManager.ADS_LEADS; chips++){
		if(ADSManager.use_neg_inputs[chips]){
			for (uint8_t chan=1; chan <= 8; chan++) {
				ADSManager.changeChannelLeadOffDetection(chan, ON, NCHAN, chips);
				ADSManager.changeChannelLeadOffDetection(chan, OFF, PCHAN, chips);
			}
		}else{
			for (uint8_t chan=1; chan <= 8; chan++) {
				ADSManager.changeChannelLeadOffDetection(chan, ON, PCHAN, chips);
				ADSManager.changeChannelLeadOffDetection(chan, OFF, NCHAN, chips);
			}
		}
	}
}

//********************************************************************************************
//! \brief Print all register's name and value for all chips.
//
//********************************************************************************************
void ads1299_printAllchipRegister(){
	for(uint8_t chips = 0; chips < ADSManager.ADS_LEADS; chips++){
		ADSManager.printAllRegisters(chips);
	}
}

//********************************************************************************************
//! \brief Send new data to uart port.
//! \no action when no new data ready.
//
//********************************************************************************************
void ads1299_update(){
    if(ads_rx_new){
	    ADSManager.updateChannelData();
	    sampleCounter++;
	    ADSManager.writeChannelDataAsUAISLab(sampleCounter);
	    ads_rx_new = false;
    }
}

//********************************************************************************************
//! \brief Start continuous data acquisition.
//
//********************************************************************************************
void ads1299_startRunning(){
	ADSManager.start();    //start the data acquisition
}

//********************************************************************************************
//! \brief Stop continuous data acquisition.
//
//********************************************************************************************
void ads1299_stopRunning(){
	ADSManager.stop();
}

//********************************************************************************************
//! \brief Configure all channel to test mode with fast and 2X amp internal test siganl.
//
//********************************************************************************************
void ads1299_activateAllChannelsToTestCondition_fast_2X(){
	ads1299_activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_FAST);
}

//********************************************************************************************
//! \brief Configure all channel to test mode with slow and 1X amp internal test siganl.
//
//********************************************************************************************
void ads1299_activateAllChannelsToTestCondition_slow_1X(){
	ads1299_activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_SLOW);
}

//********************************************************************************************
//! \brief Configure internal testing signal for all channel.
//!
//! \param[in] amplitudeCode: amplitude of excitation signal: ADSTESTSIG_AMP_1X
//!                                                        or ADSTESTSIG_AMP_2X.
//! \param[in] freqCode: frequence of excitation signal:  ADSTESTSIG_PULSE_SLOW
//!                                                    or ADSTESTSIG_PULSE_FAST.
//
//********************************************************************************************
void ads1299_activateAllChannelsToTestCondition(uint8_t testInputCode, uint8_t amplitudeCode, uint8_t freqCode){
  ads1299_stopRunning();
  ADSManager.configureInternalTestSignal(amplitudeCode,freqCode);

  //loop over all channels to change their state
  for(uint8_t chips = 0; chips < ADSManager.ADS_LEADS; chips++){
	  for (int Ichan=1; Ichan <= 8; Ichan++) {
	    ADSManager.activateChannel(Ichan, ADS_GAIN24, testInputCode, chips);  //Ichan must be [1 8]...it does not start counting from zero
	  }
  }
}
