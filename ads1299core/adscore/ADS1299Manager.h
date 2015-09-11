/*
 * ADS1299.cpp with msp430f5529
 *
 *  Created on: 2015Äê9ÔÂ9ÈÕ
 *      Author: jfanl
 */


#ifndef ____ADS1299Manager__
#define ____ADS1299Manager__

#include <ADS1299.h>

//Pick which type input will be used.
#define POSTIVE_INPUT (1)
#define NEGATIVE_INPUT (2)


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
	//********************************************************************************************
	//! \brief Initializes ADS1299 manager layer.
	//!
	//! \param[in] cb is the call back function for data ready interrupt(DADY).
	//! \param[in] ads_leads is the leads of multi-chip ads1299: ADS_LEADS_8,
	//!             ADS_LEADS_16, ADS_LEADS_24(Reserve) or ADS_LEADS_32(Reserve).
	//
	//********************************************************************************************
    void initialize(void (*cb)(void), uint8_t ads_leads);
    //********************************************************************************************
    //! \brief Initializes ADS1299 manager layer.
    //!
    //! \param[in] cb is the call back function for data ready interrupt(DADY).
    //! \param[in] ads_leads is the leads of multi-chip ads1299: ADS_LEADS_8,
    //!             ADS_LEADS_16, ADS_LEADS_24(Reserve) or ADS_LEADS_32(Reserve).
    //! \param[in] inputType: use positive or negative inputs for eeg: POSTIVE_INPUT
    //!                                                            or NEGATIVE_INPUT.
    //
    //********************************************************************************************//initialize the ADS1299 controller.  Call once.  Assumes OpenBCI_V2
    void initialize(void (*cb)(void),uint8_t ads_leads,const uint8_t inputType,bool isDaisy);              //initialize the ADS1299 controller.  Call once.  Set which version of OpenBCI you're using.
    //********************************************************************************************
    //! \brief Select eeg input type: negative input or positive input.
    //!
    //! \param[in] inputType: use positive or negative inputs for eeg: POSTIVE_INPUT
    //!                                                            or NEGATIVE_INPUT.
    //! \param[in] chips: which chip register will be read.
    //
    //********************************************************************************************
    void setInputType(const uint8_t inputType, uint8_t chips);			//Set which version of OpenBCI you're using.
    //********************************************************************************************
    //! \brief Reset chips and registers value.
    //
    //********************************************************************************************
    void reset(void);                                          //reset all the ADS1299's settings.  Call however you'd like
    //********************************************************************************************
    //! \brief Get channel active state.
    //!
    //! \param[in] N_oneRef: channel num, from 1 to 8.
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //!
    //! \return true if active, or false.
    //
    //********************************************************************************************
    bool isChannelActive(uint8_t N_oneRef,uint8_t chips);
    //********************************************************************************************
    //! \brief Active channel.
    //!
    //! \param[in] N_oneRef: channel num, from 1 to 8.
    //! \param[in] gainCode:  channel LN-PGA gain.
    //! \param[in] inputCode:  ADSINPUT_NORMAL, ADSINPUT_SHORTED OR ADSINPUT_TESTSIG.
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR
    //
    //********************************************************************************************
    void activateChannel(uint8_t N_oneRef,uint8_t gainCode,uint8_t inputCode,uint8_t chips); //setup the channel 1-8
    //********************************************************************************************
    //! \brief Deactive channel.
    //!
    //! \param[in] N_oneRef: channel num, from 1 to 8.
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //
    //********************************************************************************************
    void deactivateChannel(uint8_t N_oneRef, uint8_t chips);                           //disable given channel 1-8
    //********************************************************************************************
    //! \brief Configure lead-off detection excitation signal for all channel.
    //!
    //! \param[in] amplitudeCode: amplitude of excitation signal: LOFF_MAG_6NA, LOFF_MAG_24NA,
    //!                                                         LOFF_MAG_6UA or LOFF_MAG_24UA.
    //! \param[in] freqCode: frequence of excitation signal: LOFF_FREQ_DC, LOFF_FREQ_7p8HZ,
    //!                                                    LOFF_FREQ_31p2HZ or LOFF_FREQ_FS_4.
    //
    //********************************************************************************************
    void configureLeadOffDetection(uint8_t amplitudeCode, uint8_t freqCode);
    //********************************************************************************************
    //! \brief Enale or disable lead-off detection.
    //!
    //! \param[in] N_oneRef: channel num, from 1 to 8.
    //! \param[in] code_OFF_ON: ON for enable, OFF for disable.
    //! \param[in] code_P_N_Both: PCHAN, NCHAN, BOTHCHAN.
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR
    //
    //********************************************************************************************
    void changeChannelLeadOffDetection(uint8_t N_oneRef, uint8_t code_OFF_ON, uint8_t code_P_N_Both,uint8_t chips);
    //********************************************************************************************
    //! \brief Configure internal testing signal for all channel.
    //!
    //! \param[in] amplitudeCode: amplitude of excitation signal: ADSTESTSIG_AMP_1X
    //!                                                        or ADSTESTSIG_AMP_2X.
    //! \param[in] freqCode: frequence of excitation signal:  ADSTESTSIG_PULSE_SLOW
    //!                                                    or ADSTESTSIG_PULSE_FAST.
    //
    //********************************************************************************************
    void configureInternalTestSignal(uint8_t amplitudeCode, uint8_t freqCode);  //configure the test signal parameters
    //********************************************************************************************
    //! \brief Start continuous data acquisition.
    //
    //********************************************************************************************
    void start(void);
    //********************************************************************************************
    //! \brief Stop continuous data acquisition.
    //
    //********************************************************************************************
    void stop(void);
    //********************************************************************************************
    //! \brief Send data for data read continuous mode.
    //!
    //! \param[in] sampleNumber: sample number.
    //
    //********************************************************************************************
    void writeChannelDataAsUAISLab(uint32_t sampleNumber);
    //********************************************************************************************
    //! \brief Print all register name and its value for one chip.
    //!
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //
    //********************************************************************************************
    void printAllRegisters(uint8_t chips);
    //********************************************************************************************
    //! \brief Connect n-channel to SRB1 if not used SRB2 as reference lead.
    //!
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //
    //********************************************************************************************
    void setSRB1(uint8_t chips);
    //********************************************************************************************
    //! \brief Connect or disconnect n-channel to SRB1.
    //!
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //! \param[in] code_OFF_ON:  ON for connect, OFF for disconnect.
    //
    //********************************************************************************************
    void setSRB1(uint8_t chips, uint8_t code_OFF_ON);
    //********************************************************************************************
    //! \brief Connect or disconnect p-channel to SRB2.
    //!
    //! \param[in] N_oneRef: channel num, from 1 to 8.
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //! \param[in] code_OFF_ON:  ON for connect, OFF for disconnect.
    //
    //********************************************************************************************
    void setSRB2(uint8_t N_oneRef,uint8_t chips,uint8_t code_OFF_ON);
    //********************************************************************************************
    //! \brief Set generation according to channel active state.
    //!
    //! \param[in] N_oneRef: channel num, from 1 to 8.
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //
    //********************************************************************************************
    void alterBiasBasedOnChannelState(uint8_t N_oneRef,uint8_t chips);
    //********************************************************************************************
    //! \brief Deactive channel to generate bias.
    //!
    //! \param[in] N_oneRef: channel num, from 1 to 8.
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //
    //********************************************************************************************
    void deactivateBiasForChannel(uint8_t N_oneRef,uint8_t chips);
    //********************************************************************************************
    //! \brief Active channel to generate bias.
    //!
    //! \param[in] N_oneRef: channel num, from 1 to 8.
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //
    //********************************************************************************************
    void activateBiasForChannel(uint8_t N_oneRef,uint8_t chips);
    //********************************************************************************************
    //! \brief Enable bias generation and set generation according to channel cative state.
    //!
    //! \param[in] state: true for enable, false for disable.
    //
    //********************************************************************************************
    void setAutoBiasGeneration(bool state);
    bool use_neg_inputs[ADS1299_MAX_CHIPS];    //hold chip input type: negative input or positive
    
  private:
    bool use_channels_for_bias;                //flag for enable or disable bias generetion

    bool use_SRB2[ADS1299_MAX_CHIPS * ADS_LEADS_PER_CHIP];//mark whether or not connect to SBR2
    //********************************************************************************************
    //! \brief decide whether or not to use SRB1 as reference lead.
    //! \use SRB1 as reference if not use SRB2 as reference
    //!
    //! \param[in] chips:  chip num: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE or ADS_CHIP_FOUR.
    //!
    //! \return true if used, false if not.
    //
    //********************************************************************************************
    bool use_SRB1(uint8_t chips);
};

#endif
