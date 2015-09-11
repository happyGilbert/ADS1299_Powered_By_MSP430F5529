/*
 * ADS1299.cpp with msp430f5529
 *
 *  Created on: 2015Äê9ÔÂ9ÈÕ
 *      Author: jfanl
 */

#include <ADS1299Manager.h>
#include "msp430_clock.h"
#include "msp430_uart.h"
#include "gpio.h"

#define bitSet(data, bit) data |= (0x01<<bit)
#define bitClear(data, bit) data &= ~(0x01<<bit)

//********************************************************************************************
//! \brief Initializes ADS1299 manager layer.
//!
//! \param[in] cb is the call back function for data ready interrupt(DADY).
//! \param[in] ads_leads is the leads of multi-chip ads1299: ADS_LEADS_8,
//!             ADS_LEADS_16, ADS_LEADS_24(Reserve) or ADS_LEADS_32(Reserve).
//
//********************************************************************************************
void ADS1299Manager::initialize(void (*cb)(void), uint8_t ads_leads) {
	bool isDaisy = false;
    initialize(cb, ads_leads, NEGATIVE_INPUT, isDaisy);
}

//********************************************************************************************
//! \brief Initializes ADS1299 manager layer.
//!
//! \param[in] cb is the call back function for data ready interrupt(DADY).
//! \param[in] ads_leads is the leads of multi-chip ads1299: ADS_LEADS_8,
//!             ADS_LEADS_16, ADS_LEADS_24(Reserve) or ADS_LEADS_32(Reserve).
//! \param[in] inputType: use positive or negative inputs for eeg: POSTIVE_INPUT
//!                                                            or NEGATIVE_INPUT.
//
//********************************************************************************************
void ADS1299Manager::initialize(void (*cb)(void),uint8_t ads_leads,const uint8_t inputType,bool isDaisy){
  ADS1299::initialize(cb,ads_leads);
  msp430_delay_ms(100);
    
  verbose = false;
  for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
	  setInputType(inputType,chips);
  }
  reset();
  
  configureInternalTestSignal(ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); //set internal test signal, default amplitude, 2x speed, datasheet PDF Page 41
  

  configureLeadOffDetection(LOFF_MAG_6NA,LOFF_FREQ_DC);                 //set default state for lead off detection
}

//********************************************************************************************
//! \brief Select eeg input type: negative input or positive input.
//!
//! \param[in] inputType: use positive or negative inputs for eeg: POSTIVE_INPUT
//!                                                            or NEGATIVE_INPUT.
//! \param[in] chips: which chip register will be read.
//
//********************************************************************************************
void ADS1299Manager::setInputType(const uint8_t inputType, uint8_t chips){
  if (inputType == POSTIVE_INPUT) {

  	  use_neg_inputs[chips] = false;  	                   //mark chip input type
  	  for (uint8_t i=0; i < ADS_LEADS_PER_CHIP; i++) {
  	  	  use_SRB2[i + chips * ADS_LEADS_PER_CHIP] = false;//not connect p-channel to SRB2, p-channel will be used as eeg leads input.
  	  }                                                    //all n-channel will connet to SRB1 and wll use SRB1 as reference lead.
  } else {
  	  use_neg_inputs[chips] = true;
  	  for (uint8_t i=0; i < ADS_LEADS_PER_CHIP; i++) {
  	  	  use_SRB2[i + chips * ADS_LEADS_PER_CHIP] = true;//connect all p-channel to SRB2 and will use SRB2 asreference lead.
  	  }                                                   //n-channel will be used as eeg leads input.
  	  
  }
  
  if (use_neg_inputs==false) {
  	  ADS1299::WREG(LOFF_FLIP,0b00000000,chips);msp430_delay_ms(1);//positive input: default polarity
  } else {
  	  ADS1299::WREG(LOFF_FLIP,0b11111111,chips);msp430_delay_ms(1);//negative input: flip the polarity
  }
  
}

//********************************************************************************************
//! \brief Reset chips and registers value.
//
//********************************************************************************************
void ADS1299Manager::reset(void){
  ADS1299::SDATAC();            // exit Read Data Continuous mode first.
  ADS1299::RESET();             // send RESET command to default all registers
  ADS1299::SDATAC();            // exit Read Data Continuous mode.
  
  msp430_delay_ms(100);
    
  for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
	  for (uint8_t chan=1; chan <= ADS_LEADS_PER_CHIP; chan++) {
	    deactivateChannel(chan,chips);                          //turn off the channel.
	    changeChannelLeadOffDetection(chan,OFF,BOTHCHAN,chips); //turn off lead-off detection.
	  }
	  setSRB1(chips);                                           //set whether or not to use SRB1 as reference lead.
  }
  setAutoBiasGeneration(true);                                  //enable bias generation function.
}

//********************************************************************************************
//! \brief Deactive channel.
//!
//! \param[in] N_oneRef: channel num, from 1 to 8.
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//
//********************************************************************************************
void ADS1299Manager::deactivateChannel(uint8_t N_oneRef, uint8_t chips){
  uint8_t reg, config;
	
  if ((N_oneRef < 1) || (N_oneRef > ADS_LEADS_PER_CHIP)) return;
  
  ADS1299::SDATAC(); msp430_delay_ms(1);       // exit Read Data Continuous mode first.

  uint8_t N_zeroRef = N_oneRef-1;
  reg = CH1SET+(uint8_t)N_zeroRef;
  config = ADS1299::RREG(reg,chips); msp430_delay_ms(1);
  bitSet(config,7);                            //set bit7 of CHnSET register to power down n channel
  if (use_SRB2[N_zeroRef+chips*ADS_LEADS_PER_CHIP]) bitClear(config,3);  //disconnect SRB2
  ADS1299::WREG(reg,config,chips); msp430_delay_ms(1);
  
  alterBiasBasedOnChannelState(N_oneRef,chips);//not use this channel for generating bias.
}
    
//********************************************************************************************
//! \brief Active channel.
//!
//! \param[in] N_oneRef: channel num, from 1 to 8.
//! \param[in] gainCode:  channel LN-PGA gain.
//! \param[in] inputCode:  ADSINPUT_NORMAL, ADSINPUT_SHORTED OR ADSINPUT_TESTSIG.
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR
//
//********************************************************************************************
void ADS1299Manager::activateChannel(uint8_t N_oneRef,uint8_t gainCode,uint8_t inputCode,uint8_t chips){
  if ((N_oneRef < 1) || (N_oneRef > ADS_LEADS_PER_CHIP)) return;
  
  ADS1299::SDATAC(); msp430_delay_ms(1);                        // exit Read Data Continuous mode first.

  uint8_t N_zeroRef = N_oneRef-1;
  uint8_t configByte = 0b00000000;
  gainCode = gainCode & 0b01110000;
  configByte = configByte | gainCode;
  inputCode = inputCode & 0b00000111;
  configByte = configByte | inputCode;
  if (use_SRB2[N_zeroRef+chips*ADS_LEADS_PER_CHIP]) configByte |= 0b00001000;  //connect to SRB2 if used.
  ADS1299::WREG(CH1SET+(uint8_t)N_zeroRef,configByte,chips); msp430_delay_ms(1);

  alterBiasBasedOnChannelState(N_oneRef,chips);                //use tihs channel to generating bias.
  
  setSRB1(chips);                                              //connect to SRB1 if used

  if(chips == ADS_CHIP_ONE){//active reference buffer and bias buffer, select internal bias reference for chip one.
	  ADS1299::WREG(CONFIG3,0b11101100,chips);
  }else{                    //active reference buffer,deactive bias buffer, select internal bias reference for other chip.
	  ADS1299::WREG(CONFIG3,0b11101000,chips);
  }
  msp430_delay_ms(1);
}

//********************************************************************************************
//! \brief Get channel active state.
//!
//! \param[in] N_oneRef: channel num, from 1 to 8.
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//!
//! \return true if active, or false.
//
//********************************************************************************************
bool ADS1299Manager::isChannelActive(uint8_t N_oneRef,uint8_t chips) {
	 uint8_t N_zeroRef = N_oneRef-1;
	 uint8_t reg = CH1SET+(uint8_t)N_zeroRef;
	 uint8_t config = ADS1299::RREG(reg,chips); msp430_delay_ms(1);
	 bool chanState = ((config & 0x80)?0:1);       //bit7: 0(normal operation), 1(power down)
	 return chanState;
}

//********************************************************************************************
//! \brief Enable bias generation and set generation according to channel cative state.
//!
//! \param[in] state: true for enable, false for disable.
//
//********************************************************************************************
void ADS1299Manager::setAutoBiasGeneration(bool state) {
	use_channels_for_bias = state;
    for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
    	for (uint8_t Ichan=1; Ichan<=ADS_LEADS_PER_CHIP;Ichan++) {
    		alterBiasBasedOnChannelState(Ichan,chips);
    	}
    }
}

//********************************************************************************************
//! \brief Set generation according to channel active state.
//!
//! \param[in] N_oneRef: channel num, from 1 to 8.
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//
//********************************************************************************************
void ADS1299Manager::alterBiasBasedOnChannelState(uint8_t N_oneRef,uint8_t chips) {
	
	 if ((use_channels_for_bias==true) && (isChannelActive(N_oneRef,chips))) {
	 	 activateBiasForChannel(N_oneRef,chips);
	 } else {
	 	 deactivateBiasForChannel(N_oneRef,chips);
	 }
}
	
//********************************************************************************************
//! \brief Deactive channel to generate bias.
//!
//! \param[in] N_oneRef: channel num, from 1 to 8.
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//
//********************************************************************************************
void ADS1299Manager::deactivateBiasForChannel(uint8_t N_oneRef,uint8_t chips) {
	uint8_t N_zeroRef = N_oneRef-1; //subtracts 1 so that we're counting from 0, not 1
 	
	uint8_t reg, config;
	for (uint8_t I=0;I<2;I++) {
		if (I==0) {
			reg = BIAS_SENSP;
		} else {
			reg = BIAS_SENSN;
		}
		config = ADS1299::RREG(reg,chips); msp430_delay_ms(1);//get the current bias settings
		bitClear(config,N_zeroRef);                           //clear this channel's generation bit
		ADS1299::WREG(reg,config,chips); msp430_delay_ms(1);
	}
}

//********************************************************************************************
//! \brief Active channel to generate bias.
//!
//! \param[in] N_oneRef: channel num, from 1 to 8.
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//
//********************************************************************************************
void ADS1299Manager::activateBiasForChannel(uint8_t N_oneRef,uint8_t chips) {
	uint8_t N_zeroRef = N_oneRef-1; //subtracts 1 so that we're counting from 0, not 1
 	
	//see ADS1299 datasheet, PDF p44.
	uint8_t reg, config;
	for (uint8_t i=0; i < 2; i++) {
		reg = BIAS_SENSP;
		if (i > 0) reg = BIAS_SENSN;
		config = ADS1299::RREG(reg,chips);                    //get the current bias settings
		bitSet(config,N_zeroRef);                             //set this channel's bit
		ADS1299::WREG(reg,config,chips); msp430_delay_ms(1);  //send the modified byte back to the ADS
	}
}	

//********************************************************************************************
//! \brief Enale or disable lead-off detection.
//!
//! \param[in] N_oneRef: channel num, from 1 to 8.
//! \param[in] code_OFF_ON: ON for enable, OFF for disable.
//! \param[in] code_P_N_Both: PCHAN, NCHAN, BOTHCHAN.
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR
//
//********************************************************************************************
void ADS1299Manager::changeChannelLeadOffDetection(uint8_t N_oneRef, uint8_t code_OFF_ON, uint8_t code_P_N_Both,uint8_t chips){
  uint8_t reg, config_p, config_n;
	
  if ((N_oneRef < 1) || (N_oneRef > ADS_LEADS_PER_CHIP)) return;
  uint8_t N_zeroRef = N_oneRef-1;
  
  ADS1299::SDATAC(); msp430_delay_ms(1);      // exit Read Data Continuous mode first

  if ((code_P_N_Both == PCHAN) || (code_P_N_Both == BOTHCHAN)) {
  	  reg = LOFF_SENSP;
  	  config_p = ADS1299::RREG(reg,chips);
  	  if (code_OFF_ON == OFF) {
  	  	  bitClear(config_p,N_zeroRef);       //clear this channel's bit for disable.
  	  } else {
  	  	  bitSet(config_p,N_zeroRef); 		  //Set this channel's bit for enable.
  	  }
  	  ADS1299::WREG(reg,config_p,chips); msp430_delay_ms(1);
  }
  
  if ((code_P_N_Both == NCHAN) || (code_P_N_Both == BOTHCHAN)) {
  	  reg = LOFF_SENSN;
  	  config_n = ADS1299::RREG(reg,chips);
  	  if (code_OFF_ON == OFF) {
  	  	  bitClear(config_n,N_zeroRef);       //clear this channel's bit for disable.
  	  } else {
  	  	  bitSet(config_n,N_zeroRef); 		  //Set this channel's bit for enable.
  	  }
  	  ADS1299::WREG(reg,config_n,chips); msp430_delay_ms(1);
  }
  if(config_p || config_n){
	  ADS1299::WREG(CONFIG4,0x02,chips);  //use lead-off detection, so enable lead-off comparators for lead off detection.
  }else{
	  ADS1299::WREG(CONFIG4,0x00,chips);  //not use lead-off detection, so disable lead-off comparators.
  }
  msp430_delay_ms(1);
}

//********************************************************************************************
//! \brief Configure lead-off detection excitation signal for all channel.
//!
//! \param[in] amplitudeCode: amplitude of excitation signal: LOFF_MAG_6NA, LOFF_MAG_24NA,
//!                                                         LOFF_MAG_6UA or LOFF_MAG_24UA.
//! \param[in] freqCode: frequence of excitation signal: LOFF_FREQ_DC, LOFF_FREQ_7p8HZ,
//!                                                    LOFF_FREQ_31p2HZ or LOFF_FREQ_FS_4.
//
//********************************************************************************************
void ADS1299Manager::configureLeadOffDetection(uint8_t amplitudeCode, uint8_t freqCode)
{
	amplitudeCode &= 0b00001100;
	freqCode &= 0b00000011;
	
	uint8_t reg, config;
	reg = LOFF;
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
		config = ADS1299::RREG(reg,chips);

		config &= 0b11110000;
		config |= amplitudeCode;
		config |= freqCode;

		ADS1299::WREG(reg,config,chips);
	}
	msp430_delay_ms(1);
}

//********************************************************************************************
//! \brief Connect n-channel to SRB1 if not used SRB2 as reference lead.
//!
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//
//********************************************************************************************
void ADS1299Manager::setSRB1(uint8_t chips) {
	if (use_SRB1(chips)) {
		ADS1299::WREG(MISC1,0b00100000,chips);
	} else {
		ADS1299::WREG(MISC1,0b00000000,chips);
	}
	msp430_delay_ms(1);
}

//********************************************************************************************
//! \brief Connect or disconnect n-channel to SRB1.
//!
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//! \param[in] code_OFF_ON:  ON for connect, OFF for disconnect.
//
//********************************************************************************************
void ADS1299Manager::setSRB1(uint8_t chips, uint8_t code_OFF_ON) {
	//proceed...first, disable any data collection
    ADS1299::SDATAC(); msp430_delay_ms(1);      // exit Read Data Continuous mode first.
	if (code_OFF_ON == ON) {
		ADS1299::WREG(MISC1,0b00100000,chips);
	} else {
		ADS1299::WREG(MISC1,0b00000000,chips);
	}
	msp430_delay_ms(1);
}
//********************************************************************************************
//! \brief Connect or disconnect p-channel to SRB2.
//!
//! \param[in] N_oneRef: channel num, from 1 to 8.
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//! \param[in] code_OFF_ON:  ON for connect, OFF for disconnect.
//
//********************************************************************************************
void ADS1299Manager::setSRB2(uint8_t N_oneRef,uint8_t chips,uint8_t code_OFF_ON) {
	uint8_t reg, config;
	if ((N_oneRef < 1) || (N_oneRef > ADS_LEADS_PER_CHIP)) return;

	ADS1299::SDATAC(); msp430_delay_ms(1);      // exit Read Data Continuous mode first.
	uint8_t N_zeroRef = N_oneRef - 1;
	reg = CH1SET+(uint8_t)N_zeroRef;
	config = ADS1299::RREG(reg,chips); msp430_delay_ms(1);
	if (code_OFF_ON == ON) {
		bitSet(config,3);
		use_SRB2[N_zeroRef + chips * ADS_LEADS_PER_CHIP] = true;
	} else {
		bitClear(config,3);
		use_SRB2[N_zeroRef + chips * ADS_LEADS_PER_CHIP] = false;
	}
	ADS1299::WREG(reg,config,chips); msp430_delay_ms(1);
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
void ADS1299Manager::configureInternalTestSignal(uint8_t amplitudeCode, uint8_t freqCode)
{
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
	    if (amplitudeCode == ADSTESTSIG_NOCHANGE) amplitudeCode = (ADS1299::RREG(CONFIG2,chips) & (0b00000100));
	    if (freqCode == ADSTESTSIG_NOCHANGE) freqCode = (ADS1299::RREG(CONFIG2,chips) & (0b00000011));
	    freqCode &= 0b00000011;
	    amplitudeCode &= 0b00000100;
	    uint8_t message = 0b11010000 | freqCode | amplitudeCode;

		ADS1299::WREG(CONFIG2,message,chips);
	}
	msp430_delay_ms(1);
	

}

//********************************************************************************************
//! \brief Start continuous data acquisition.
//
//********************************************************************************************
void ADS1299Manager::start(void)
{
    ADS1299::RDATAC(); msp430_delay_ms(1);           // enter Read Data Continuous mode.
    ADS1299::START();                                //start the data acquisition.
}

//********************************************************************************************
//! \brief Stop continuous data acquisition.
//
//********************************************************************************************
void ADS1299Manager::stop(void)
{
    ADS1299::STOP(); msp430_delay_ms(1);            //stop the data acquisition.
    ADS1299::SDATAC(); msp430_delay_ms(1);          // exit Read Data Continuous mode.
}

//********************************************************************************************
//! \brief Send data for data read continuous mode.
//!
//! \param[in] sampleNumber: sample number.
//
//********************************************************************************************
void ADS1299Manager::writeChannelDataAsUAISLab(uint32_t sampleNumber){
	uint32_t val;
	uint8_t packet, *val_ptr = (uint8_t *)(&val), state;
	packet = PCKT_START;
	msp430_uart_write(&packet,1);                   // Write start byte
	packet = PCKT_EEG;
	msp430_uart_write(&packet,1);                   // Write eeg byte

	uint8_t payloadBytes =
			(uint8_t)((1+ADS_LEADS_PER_CHIP*ADS_LEADS)*4+ADS_LEADS);
	msp430_uart_write(&payloadBytes, 1);           //write the length of the payload(samplenumber+lead-off state+eeg data)

	val = sampleNumber;                            //write the sample number,can used to check whether or not lost packet.
	msp430_uart_write(val_ptr, 4);

	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
		if(use_neg_inputs[chips]){                 //write lead-off detection result.
			state = (stat[chips] & 0x00FF);
		}else{
			state = ((stat[chips]>>8) & 0x00FF);
		}
		msp430_uart_write(&state, 1);
	}

	for (uint8_t chan = 0; chan < (ADS_LEADS * ADS_LEADS_PER_CHIP); chan++ ){
		val = channelData[chan];                   // get the real EEG data for this channel
		msp430_uart_write(val_ptr, 4);
	}
	packet = PCKT_END;
	msp430_uart_write(&packet,1);                  // Write end byte

	//msp430_uart_flush();                         // wait transfer complete.
}

//********************************************************************************************
//! \brief Print all register name and its value for one chip.
//!
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//
//********************************************************************************************
void ADS1299Manager::printAllRegisters(uint8_t chips)
{
	bool prevVerboseState = verbose;
	
        verbose = true;
        ADS1299::RREGS(0x00,0x10,chips);     // write the first registers
        msp430_delay_ms(100);  //stall to let all that data get read by the PC
        ADS1299::RREGS(0x11,0x17-0x11,chips);     // write the rest
        verbose = prevVerboseState;
}

//********************************************************************************************
//! \brief decide whether or not to use SRB1 as reference lead.
//! \use SRB1 as reference if not use SRB2 as reference
//!
//! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
//!
//! \return true if used, false if not.
//
//********************************************************************************************
inline bool ADS1299Manager::use_SRB1(uint8_t chips) {
	for (uint8_t Ichan=0; Ichan < ADS_LEADS_PER_CHIP; Ichan++) {
		if (use_SRB2[Ichan+chips*ADS_LEADS_PER_CHIP]) {
			return false;
		}
	}
	return true;
}
			
