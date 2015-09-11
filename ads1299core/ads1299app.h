/*
 * ads1299app.h
 *
 *  Created on: 2015Äê9ÔÂ10ÈÕ
 *      Author: jfanl
 */

#ifndef ADS1299APP_H_
#define ADS1299APP_H_
#include "ADS1299Manager.h"

#define GAIN ADS_GAIN24    //gain for all channel's LN-PGA

//********************************************************************************************
//! \brief Initializes ADS1299 app layer.
//!
//! \param[in] ads_leads is the leads of multi-chip ads1299: ADS_LEADS_8,
//!             ADS_LEADS_16, ADS_LEADS_24(Reserve) or ADS_LEADS_32(Reserve).
//
//********************************************************************************************
void ads1299_init(uint8_t ads_leads);
//********************************************************************************************
//! \brief Send new data to uart port.
//! \no action when no new data ready.
//
//********************************************************************************************
void ads1299_update();
//********************************************************************************************
//! \brief Start continuous data acquisition.
//
//********************************************************************************************
void ads1299_startRunning();
//********************************************************************************************
//! \brief Stop continuous data acquisition.
//
//********************************************************************************************
void ads1299_stopRunning();

//********************************************************************************************
//! \brief Configure all channel to normal data continuous sampling mode.
//
//********************************************************************************************
void ads1299_activeAllChannelToNormalOperation();
//********************************************************************************************
//! \brief Active channel lead-off detection.
//! \for negative input use n-channel lead-off detection.
//! \for positive input use p-channel lead-off detection.]
//! \not available in internal testing siganl mode.
//
//********************************************************************************************
void ads1299_activeLeadOFFDetection();
//********************************************************************************************
//! \brief Print all register's name and value for all chips.
//
//********************************************************************************************
void ads1299_printAllchipRegister();

//********************************************************************************************
//! \brief Configure all channel to test mode with fast and 2X amp internal test siganl.
//
//********************************************************************************************
void ads1299_activateAllChannelsToTestCondition_fast_2X();
//********************************************************************************************
//! \brief Configure all channel to test mode with slow and 1X amp internal test siganl.
//
//********************************************************************************************
void ads1299_activateAllChannelsToTestCondition_slow_1X();
//********************************************************************************************
//! \brief Configure internal testing signal for all channel.
//!
//! \param[in] amplitudeCode: amplitude of excitation signal: ADSTESTSIG_AMP_1X
//!                                                        or ADSTESTSIG_AMP_2X.
//! \param[in] freqCode: frequence of excitation signal:  ADSTESTSIG_PULSE_SLOW
//!                                                    or ADSTESTSIG_PULSE_FAST.
//
//********************************************************************************************
void ads1299_activateAllChannelsToTestCondition(uint8_t testInputCode, uint8_t amplitudeCode, uint8_t freqCode);

#endif /* ADS1299APP_H_ */
