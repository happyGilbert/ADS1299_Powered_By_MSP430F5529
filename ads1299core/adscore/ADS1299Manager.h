
//
//  ADS1299Manager.h
//  Part of the Arduino Library for the ADS1299 Shield
//  Created by Chip Audette, Fall 2013
//


#ifndef ____ADS1299Manager__
#define ____ADS1299Manager__

#include <ADS1299.h>

//Pick which version of OpenBCI you have
#define POSTIVE_INPUT (1)    //Sept 2013
#define NEGATIVE_INPUT (2)    //Oct 24, 2013
#define OPENBCI_NCHAN_PER_BOARD (8)  // number of EEG channels

//gainCode choices
#define ADS_GAIN01 (0b00000000)
#define ADS_GAIN02 (0b00010000)
#define ADS_GAIN04 (0b00100000)
#define ADS_GAIN06 (0b00110000)
#define ADS_GAIN08 (0b01000000)
#define ADS_GAIN12 (0b01010000)
#define ADS_GAIN24 (0b01100000)

//inputCode choices
#define ADSINPUT_NORMAL (0b00000000)
#define ADSINPUT_SHORTED (0b00000001)
#define ADSINPUT_TESTSIG (0b00000101)

//test signal choices...ADS1299 datasheet page 41
#define ADSTESTSIG_AMP_1X (0b00000000)
#define ADSTESTSIG_AMP_2X (0b00000100)
#define ADSTESTSIG_PULSE_SLOW (0b00000000)
#define ADSTESTSIG_PULSE_FAST (0b00000001)
#define ADSTESTSIG_DCSIG (0b00000011)
#define ADSTESTSIG_NOCHANGE (0b11111111)

//Lead-off signal choices
#define LOFF_MAG_6NA (0b00000000)
#define LOFF_MAG_24NA (0b00000100)
#define LOFF_MAG_6UA (0b00001000)
#define LOFF_MAG_24UA (0b00001100)
#define LOFF_FREQ_DC (0b00000000)
#define LOFF_FREQ_7p8HZ (0b00000001)
#define LOFF_FREQ_31p2HZ (0b00000010)
#define LOFF_FREQ_FS_4 (0b00000011)
#define PCHAN (1)
#define NCHAN (2)
#define BOTHCHAN (3)

#define OFF (0)
#define ON (1)

//binary communication codes for each packet
#define PCKT_START 0xA0
#define PCKT_EEG 0x30
#define PCKT_END 0xC0

class ADS1299Manager : public ADS1299 {
  public:
    void initialize(void (*cb)(void));                                     //initialize the ADS1299 controller.  Call once.  Assumes OpenBCI_V2
    void initialize(void (*cb)(void),uint8_t version,bool isDaisy);              //initialize the ADS1299 controller.  Call once.  Set which version of OpenBCI you're using.
    void setInputType(const uint8_t inputType);			//Set which version of OpenBCI you're using.
    void reset(void);                                          //reset all the ADS1299's settings.  Call however you'd like
    bool isChannelActive(uint8_t N_oneRef);
    void activateChannel(uint8_t N_oneRef, uint8_t gainCode,uint8_t inputCode); //setup the channel 1-8
    void deactivateChannel(uint8_t N_oneRef);                            //disable given channel 1-8
    void configureLeadOffDetection(uint8_t amplitudeCode, uint8_t freqCode);  //configure the lead-off detection signal parameters
    void changeChannelLeadOffDetection(uint8_t N_oneRef, uint8_t code_OFF_ON, uint8_t code_P_N_Both);
    void configureInternalTestSignal(uint8_t amplitudeCode, uint8_t freqCode);  //configure the test signal parameters
    void start(void);
    void stop(void);
    void writeChannelDataAsUAISLab(uint16_t sampleNumber);
    void printAllRegisters(void);
    void setSRB1(bool desired_state);
    void alterBiasBasedOnChannelState(uint8_t N_oneRef);
    void deactivateBiasForChannel(uint8_t N_oneRef);
    void activateBiasForChannel(uint8_t N_oneRef);
    void setAutoBiasGeneration(bool state);
    
    
  private:
    bool use_neg_inputs;
    bool use_SRB2[OPENBCI_NCHAN_PER_BOARD];
    bool use_channels_for_bias;
    bool use_SRB1(void);
};

#endif
