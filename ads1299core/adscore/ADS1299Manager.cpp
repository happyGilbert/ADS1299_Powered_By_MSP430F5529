


#include <ADS1299Manager.h>
#include "msp430_clock.h"
#include "msp430_uart.h"
#include "gpio.h"

#define bitSet(data, bit) data |= (0x01<<bit)
#define bitClear(data, bit) data &= ~(0x01<<bit)

typedef long int32;
//typedef byte uint8_t;


void ADS1299Manager::initialize(void (*cb)(void)) {
	bool isDaisy = false;
    initialize(cb,NEGATIVE_INPUT,isDaisy);
}

//Initilize the ADS1299 controller...call this once
void ADS1299Manager::initialize(void (*cb)(void),const uint8_t inputType,bool isDaisy)
{
  ADS1299::initialize(cb);
  msp430_delay_ms(100);
    
  verbose = false;              // when verbose is true, there will be Serial feedback
  setInputType(inputType);
  reset();
  
  //set default state for internal test signal
  configureInternalTestSignal(ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); //set internal test signal, default amplitude, 2x speed, datasheet PDF Page 41
  
  //set default state for lead off detection
  configureLeadOffDetection(LOFF_MAG_6NA,LOFF_FREQ_DC);
};

//set which version of OpenBCI we're using.  This affects whether we use the 
//positive or negative inputs.  It affects whether we use SRB1 or SRB2 for the
//referenece signal.  Finally, it affects whether the lead_off signals are
//flipped or not.
void ADS1299Manager::setInputType(const uint8_t inputType)
{
  if (inputType == POSTIVE_INPUT) {
  	  //set whether to use positive or negative inputs
  	  use_neg_inputs = false;
  	  
  	  //set SRB2
  	  for (uint8_t i=0; i < OPENBCI_NCHAN_PER_BOARD; i++) {
  	  	  use_SRB2[i] = false;
  	  }
  } else {
  	  //set whether to use positive or negative inputs
  	  use_neg_inputs = true;
  	  
  	  //set SRB
  	  for (uint8_t i=0; i < OPENBCI_NCHAN_PER_BOARD; i++) {
  	  	  use_SRB2[i] = true;
  	  }
  	  
  }
  
  //set whether or not to flip the polarity of the lead_off drive based
  //on whether we're sensing the positive or negative inputs
  if (use_neg_inputs==false) {
  	  //we're using positive.  Set to default polarity
  	  ADS1299::WREG(LOFF_FLIP,0b00000000);msp430_delay_ms(1);  //set all channels to zero
  } else {
  	  //we're using negative.  flip the polarity
  	  ADS1299::WREG(LOFF_FLIP,0b11111111);msp430_delay_ms(1);
  }
  
}

//reset all the ADS1299's settings.  Call however you'd like.  Stops all data acquisition
void ADS1299Manager::reset(void)
{
  ADS1299::SDATAC();            // exit Read Data Continuous mode to communicate with ADS
  ADS1299::RESET();             // send RESET command to default all registers
  ADS1299::SDATAC();            // exit Read Data Continuous mode to communicate with ADS
  
  msp430_delay_ms(100);
    
  // turn off all channels
  for (uint8_t chan=1; chan <= OPENBCI_NCHAN_PER_BOARD; chan++) {
    deactivateChannel(chan);  //turn off the channel
    changeChannelLeadOffDetection(chan,OFF,BOTHCHAN); //turn off any impedance monitoring
  }
  
  setSRB1(use_SRB1());  //set whether SRB1 is active or not
  setAutoBiasGeneration(true); //configure ADS1299 so that bias is generated based on channel state
};


//deactivate the given channel...note: stops data colleciton to issue its commands
//  N_oneRef is the channel number: 1-8
// 
void ADS1299Manager::deactivateChannel(uint8_t N_oneRef)
{
  uint8_t reg, config;
	
  //check the inputs
  if ((N_oneRef < 1) || (N_oneRef > OPENBCI_NCHAN_PER_BOARD)) return;
  
  //proceed...first, disable any data collection
  ADS1299::SDATAC(); msp430_delay_ms(1);      // exit Read Data Continuous mode to communicate with ADS

  //shut down the channel
  uint8_t N_zeroRef = N_oneRef-1;  //subtracts 1 so that we're counting from 0, not 1
  reg = CH1SET+(uint8_t)N_zeroRef;
  config = ADS1299::RREG(reg); msp430_delay_ms(1);
  bitSet(config,7);  //left-most bit (bit 7) = 1, so this shuts down the channel
  if (use_SRB2[N_zeroRef]) bitClear(config,3);  //bit 3 = 0 disconnects SRB2
  ADS1299::WREG(reg,config); msp430_delay_ms(1);
  
  //set how this channel affects the bias generation...
  alterBiasBasedOnChannelState(N_oneRef);
}; 
    
        
//Active a channel in single-ended mode  
//  N is 1 through 8
//  gainCode is defined in the macros in the header file
//  inputCode is defined in the macros in the header file
void ADS1299Manager::activateChannel(uint8_t N_oneRef,uint8_t gainCode,uint8_t inputCode)
{
	
   //check the inputs
  if ((N_oneRef < 1) || (N_oneRef > OPENBCI_NCHAN_PER_BOARD)) return;
  
  //proceed...first, disable any data collection
  ADS1299::SDATAC(); msp430_delay_ms(1);      // exit Read Data Continuous mode to communicate with ADS

  //active the channel using the given gain.  Set MUX for normal operation
  //see ADS1299 datasheet, PDF p44
  uint8_t N_zeroRef = N_oneRef-1;  //shift down by one
  uint8_t configByte = 0b00000000;  //left-most zero (bit 7) is to activate the channel
  gainCode = gainCode & 0b01110000;  //bitwise AND to get just the bits we want and set the rest to zero
  configByte = configByte | gainCode; //bitwise OR to set just the gain bits high or low and leave the rest alone
  inputCode = inputCode & 0b00000111;  //bitwise AND to get just the bits we want and set the rest to zero
  configByte = configByte | inputCode; //bitwise OR to set just the gain bits high or low and leave the rest alone
  if (use_SRB2[N_zeroRef]) configByte |= 0b00001000;  //set the SRB2 flag...p44 in the data sheet
  ADS1299::WREG(CH1SET+(uint8_t)N_zeroRef,configByte); msp430_delay_ms(1);

  //add this channel to the bias generation
  alterBiasBasedOnChannelState(N_oneRef);
  
  // // Now, these actions are necessary whenever there is at least one active channel
  // // though they don't strictly need to be done EVERY time we activate a channel.
  // // just once after the reset.
  
  //activate SRB1 as the Negative input for all channels, if needed
  setSRB1(use_SRB1());

  //Finalize the bias setup...activate buffer and use internal reference for center of bias creation, datasheet PDF p42
  ADS1299::WREG(CONFIG3,0b11101100); msp430_delay_ms(1);
};

//note that N here one-referenced (ie [1...N]), not [0...N-1]
//return true for active.
bool ADS1299Manager::isChannelActive(uint8_t N_oneRef) {
	 uint8_t N_zeroRef = N_oneRef-1;  //subtracts 1 so that we're counting from 0, not 1
	 
	 //get whether channel is active or not
	 uint8_t reg = CH1SET+(uint8_t)N_zeroRef;
	 uint8_t config = ADS1299::RREG(reg); msp430_delay_ms(1);
	 bool chanState = ((config & 0x80)?0:1);   //bit7: 0(normal operation), 1(power down)
	 return chanState;
}

void ADS1299Manager::setAutoBiasGeneration(bool state) {
	use_channels_for_bias = state;
	
	//step through the channels are recompute the bias state
	for (uint8_t Ichan=1; Ichan<=OPENBCI_NCHAN_PER_BOARD;Ichan++) {
		alterBiasBasedOnChannelState(Ichan);
	}
}

//note that N here one-referenced (ie [1...N]), not [0...N-1]
void ADS1299Manager::alterBiasBasedOnChannelState(uint8_t N_oneRef) {
	
	 if ((use_channels_for_bias==true) && (isChannelActive(N_oneRef))) {
	 	 //activate this channel's bias
	 	 activateBiasForChannel(N_oneRef);
	 } else {
	 	 deactivateBiasForChannel(N_oneRef);
	 }
}
	

void ADS1299Manager::deactivateBiasForChannel(uint8_t N_oneRef) {
	uint8_t N_zeroRef = N_oneRef-1; //subtracts 1 so that we're counting from 0, not 1
 	
	//deactivate this channel's bias...both positive and negative
	//see ADS1299 datasheet, PDF p44.
	uint8_t reg, config;
	for (uint8_t I=0;I<2;I++) {
		if (I==0) {
			reg = BIAS_SENSP;
		} else {
			reg = BIAS_SENSN;
		}
		config = ADS1299::RREG(reg); msp430_delay_ms(1);//get the current bias settings
		bitClear(config,N_zeroRef);          //clear this channel's bit to remove from bias generation
		ADS1299::WREG(reg,config); msp430_delay_ms(1);  //send the modified byte back to the ADS
	}
}
void ADS1299Manager::activateBiasForChannel(uint8_t N_oneRef) {
	uint8_t N_zeroRef = N_oneRef-1; //subtracts 1 so that we're counting from 0, not 1
 	
	//see ADS1299 datasheet, PDF p44.
	//per Chip's experiments, if using the P inputs, just include the P inputs
	//per Joel's experiements, if using the N inputs, include both P and N inputs
	uint8_t reg, config;
	uint8_t nLoop = 1;  if (use_neg_inputs) nLoop=2;
	for (uint8_t i=0; i < nLoop; i++) {
		reg = BIAS_SENSP;
		if (i > 0) reg = BIAS_SENSN;
		config = ADS1299::RREG(reg); //get the current bias settings
		bitSet(config,N_zeroRef);                   //set this channel's bit
		ADS1299::WREG(reg,config); msp430_delay_ms(1);  //send the modified byte back to the ADS
	}
}	


//change the given channel's lead-off detection state...note: stops data colleciton to issue its commands
//  N is the channel number: 1-8
// 
void ADS1299Manager::changeChannelLeadOffDetection(uint8_t N_oneRef, uint8_t code_OFF_ON, uint8_t code_P_N_Both)
{
  uint8_t reg, config_p, config_n;
	
  //check the inputs
  if ((N_oneRef < 1) || (N_oneRef > OPENBCI_NCHAN_PER_BOARD)) return;
  uint8_t N_zeroRef = N_oneRef-1;  //shift down by one
  
  //proceed...first, disable any data collection
  ADS1299::SDATAC(); msp430_delay_ms(1);      // exit Read Data Continuous mode to communicate with ADS

  if ((code_P_N_Both == PCHAN) || (code_P_N_Both == BOTHCHAN)) {
  	  //shut down the lead-off signal on the positive side
  	  reg = LOFF_SENSP;  //are we using the P inptus or the N inputs?
  	  config_p = ADS1299::RREG(reg); //get the current lead-off settings
  	  if (code_OFF_ON == OFF) {
  	  	  bitClear(config_p,N_zeroRef);                   //clear this channel's bit
  	  } else {
  	  	  bitSet(config_p,N_zeroRef); 			  //clear this channel's bit
  	  }
  	  ADS1299::WREG(reg,config_p); msp430_delay_ms(1);  //send the modified byte back to the ADS
  }
  
  if ((code_P_N_Both == NCHAN) || (code_P_N_Both == BOTHCHAN)) {
  	  //shut down the lead-off signal on the negative side
  	  reg = LOFF_SENSN;  //are we using the P inptus or the N inputs?
  	  config_n = ADS1299::RREG(reg); //get the current lead-off settings
  	  if (code_OFF_ON == OFF) {
  	  	  bitClear(config_n,N_zeroRef);                   //clear this channel's bit
  	  } else {
  	  	  bitSet(config_n,N_zeroRef); 			  //clear this channel's bit
  	  }           //set this channel's bit
  	  ADS1299::WREG(reg,config_n); msp430_delay_ms(1);  //send the modified byte back to the ADS
  }
  if(config_p || config_n){
	  ADS1299::WREG(CONFIG4,0x02);                      //enable lead-off comparators for lead off detection.
  }else{
	  ADS1299::WREG(CONFIG4,0x00);
  }
  msp430_delay_ms(1);
}; 

void ADS1299Manager::configureLeadOffDetection(uint8_t amplitudeCode, uint8_t freqCode)
{
	amplitudeCode &= 0b00001100;  //only these two bits should be used
	freqCode &= 0b00000011;  //only these two bits should be used
	
	//get the current configuration of he byte
	uint8_t reg, config;
	reg = LOFF;
	config = ADS1299::RREG(reg); //get the current bias settings
	
	//reconfigure the byte to get what we want
	config &= 0b11110000;  //clear out the last four bits
	config |= amplitudeCode;  //set the amplitude
	config |= freqCode;    //set the frequency
	
	//send the config byte back to the hardware
	ADS1299::WREG(reg,config); msp430_delay_ms(1);  //send the modified byte back to the ADS
	
}

void ADS1299Manager::setSRB1(bool desired_state) {
	if (desired_state) {
		ADS1299::WREG(MISC1,0b00100000); msp430_delay_ms(1);  //ADS1299 datasheet, PDF p46
	} else {
		ADS1299::WREG(MISC1,0b00000000); msp430_delay_ms(1);  //ADS1299 datasheet, PDF p46
	}
}




//Configure the test signals that can be inernally generated by the ADS1299
void ADS1299Manager::configureInternalTestSignal(uint8_t amplitudeCode, uint8_t freqCode)
{
	if (amplitudeCode == ADSTESTSIG_NOCHANGE) amplitudeCode = (ADS1299::RREG(CONFIG2) & (0b00000100));
	if (freqCode == ADSTESTSIG_NOCHANGE) freqCode = (ADS1299::RREG(CONFIG2) & (0b00000011));
	freqCode &= 0b00000011;  //only the last two bits should be used
	amplitudeCode &= 0b00000100;  //only this bit should be used
	uint8_t message = 0b11010000 | freqCode | amplitudeCode;  //compose the code
	
	ADS1299::WREG(CONFIG2,message); msp430_delay_ms(1);
	
    //ADS1299::WREG(CONFIG2,0b11010000);delay(1);   //set internal test signal, default amplitude, default speed, datasheet PDF Page 41
    //ADS1299::WREG(CONFIG2,0b11010001);delay(1);   //set internal test signal, default amplitude, 2x speed, datasheet PDF Page 41
    //ADS1299::WREG(CONFIG2,0b11010101);delay(1);   //set internal test signal, 2x amplitude, 2x speed, datasheet PDF Page 41
    //ADS1299::WREG(CONFIG2,0b11010011); delay(1);  //set internal test signal, default amplitude, at DC, datasheet PDF Page 41
    //ADS1299::WREG(CONFIG3,0b01101100); delay(1);  //use internal reference for center of bias creation, datasheet PDF p42
}
 
//Start continuous data acquisition
void ADS1299Manager::start(void)
{
    ADS1299::RDATAC(); msp430_delay_ms(1);           // enter Read Data Continuous mode
    ADS1299::START();    //start the data acquisition
}

  
//Stop the continuous data acquisition
void ADS1299Manager::stop(void)
{
    ADS1299::STOP(); msp430_delay_ms(1);   //start the data acquisition
    ADS1299::SDATAC(); msp430_delay_ms(1);      // exit Read Data Continuous mode to communicate with ADS
}

void ADS1299Manager::writeChannelDataAsUAISLab(uint16_t sampleNumber)
{
//	uint8_t str[] = ", ", datbit;
//	for(uint8_t j = 0; j<16; j++){
//		datbit = bitRead(stat_1, 15-j);
//		msp430_uart_write(&datbit, 1);
//		if(j!=15) msp430_uart_write(str, 2);
//	}
//	str[0] = '\r';
//	str[1] = '\n';
//	msp430_uart_write(str, 2);

	int32_t val;
	uint8_t packet, *val_ptr = (uint8_t *)(&val);
    stat_2 = 0;
	packet = PCKT_START;
	msp430_uart_write(&packet,1);                   // Write start byte
	packet = PCKT_EEG;
	msp430_uart_write(&packet,1);                     // Write eeg byte

	uint8_t payloadBytes =
			(uint8_t)((1+8)*4);  //length of data payload, bytes
	msp430_uart_write(&payloadBytes, 1);                          //write the length of the payload

	if(use_neg_inputs){                                           //write lead-off detection result, if not disabled
		val =((stat_1<<8) & 0xFF00) + (stat_2 & 0x00FF);
	}else{
		val =(stat_1 & 0xFF00) + ((stat_2>>8) & 0x00FF);
	}
	val = (val<<16) + sampleNumber;                               //write the sample number, if not disabled
	msp430_uart_write(val_ptr, 4);


	for (uint8_t chan = 0; chan < (8); chan++ ){          //write each channel
		val = channelData[chan];                                  //get the real EEG data for this channel
		msp430_uart_write(val_ptr, 4);                            //4 bytes long
	}
	packet = PCKT_END;
	msp430_uart_write(&packet,1);                     // Write end byte

	// force everything out
	//msp430_uart_flush();
};

//print out the state of all the control registers
void ADS1299Manager::printAllRegisters(void)   
{
	bool prevVerboseState = verbose;
	
        verbose = true;
        ADS1299::RREGS(0x00,0x10);     // write the first registers
        msp430_delay_ms(100);  //stall to let all that data get read by the PC
        ADS1299::RREGS(0x11,0x17-0x11);     // write the rest
        verbose = prevVerboseState;
}

//only use SRB1 if all use_SRB2 are set to false
bool ADS1299Manager::use_SRB1(void) {
	for (uint8_t Ichan=0; Ichan < OPENBCI_NCHAN_PER_BOARD; Ichan++) {
		if (use_SRB2[Ichan]) {
			return false;
		}
	}
	return true;
}
			
