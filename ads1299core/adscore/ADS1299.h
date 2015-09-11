/*
 * ADS1299.cpp with msp430f5529
 *
 *  Created on: 2015Äê9ÔÂ8ÈÕ
 *      Author: jfanl
 */

#ifndef ____ADS1299__
#define ____ADS1299__

#include <stdio.h>
#include <stdint.h>
#include "Definitions.h"

#define ADS_LEADS_PER_CHIP (8)
#define ADS1299_MAX_CHIPS (2)
#define ADS_CHIP_ONE   (0)
#define ADS_CHIP_TWO   (1)     //available for ADS1299_MAX_CHIPS > 1
//#define ADS_CHIP_THREE (2)   //available for ADS1299_MAX_CHIPS > 2
//#define ADS_CHIP_FOUR  (3)   //available for ADS1299_MAX_CHIPS > 3
#define ADS_LEADS_8  (1)
#define ADS_LEADS_16 (2)   //available for ADS1299_MAX_CHIPS > 1
//#define ADS_LEADS_24 (3)   //available for ADS1299_MAX_CHIPS > 2
//#define ADS_LEADS_32 (4)   //available for ADS1299_MAX_CHIPS > 3

class ADS1299 {
public:
	//********************************************************************************************
	//! \brief Initializes ADS1299.
	//!
	//! pin map: MSP430F5529  <--->  ADS1299
	//!          P2.7         <--->  SCLK --|
	//!          P3.3         <--->  SIMO   |---USCI A0 SPI£º4MHZ
	//!          P3.4         <--->  SOMI --|
	//!          P6.0         <--->  CS FOR CHIP ONE
	//!          P6.1         <--->  CS FOR CHIP TWO
	//!          P6.2         <--->  CS FOR CHIP THREE(Reserve, must re-define ADS1299_MAX_CHIPS)
	//!          P6.3         <--->  CS FOR CHIP FOUR (Reserve, must re-define ADS1299_MAX_CHIPS)
	//!          P3.2         <--->  RST
	//!          P4.0         <--->  START
	//!          P2.1         <--->  DRDY(interrupt)
	//!
	//! \param[in] cb is the call back function for data ready interrupt(DADY).
	//! \param[in] ads_leads is the leads of multi-chip ads1299: ADS_LEADS_8,
	//!             ADS_LEADS_16, ADS_LEADS_24(Reserve) or ADS_LEADS_32(Reserve).
	//
	//********************************************************************************************
    void initialize(void (*cb)(void), uint8_t ads_leads);
    
    /*System Commands, command will send to all chip in multi-chip ads1299*/
    void WAKEUP();                  //wake up chips which stay in standby mode
    void STANDBY();                 // get into standby mode
    void RESET();                   // reset chip and all the registers to default settings
    void START();                   //start data conversion
    void STOP();                    //stop data conversion
    
    /*Data Read Commands*/
    void RDATAC();                  //enable read data continuous mode
    void SDATAC();                  //stop read data continuous mode
    void RDATA();                   //read data by command: single-shot conversion
    
    /* Register Read/Write Commands for multi-chips system.*/
    //********************************************************************************************
    //! \brief Get chip's ID for checking chip and communication function.
	//!
	//! \param[in] chips: point which chip ID to read: ADS_CHIP_ONE, ADS_CHIP_TWO, ADS_CHIP_THREE
    //!                                                or ADS_CHIP_FOUR
    //
    //********************************************************************************************
    uint8_t getDeviceID(uint8_t chips);
    //********************************************************************************************
    //! \brief Reads one register value.
	//!
	//! \param[in] _address: the address of register will be read.
	//! \param[in] chips: which chip register will be read.
    //
    //********************************************************************************************
    uint8_t RREG(uint8_t _address, uint8_t chips);
    //********************************************************************************************
    //! \brief Read more than one register starting at _address.
	//!
	//! \param[in] _address: the start address of register will be read.
    //! \param[in] _numRegistersMinusOne: number of the registers to read - 1
	//! \param[in] chips: which chip register will be read.
    //
    //********************************************************************************************
    void RREGS(uint8_t _address, uint8_t _numRegistersMinusOne, uint8_t chips);
    //********************************************************************************************
    //! \brief Write one register.
	//!
	//! \param[in] _address: the address of register will be written.
	//! \param[in] _value: the data will be written into register.
	//! \param[in] chips: which chip register will be written.
    //
    //********************************************************************************************
    void WREG(uint8_t _address, uint8_t _value, uint8_t chips);
    //********************************************************************************************
    //! \brief Write one register.
    //!
	//! \The data will be written into register are storaged in regData[]
    //!
	//! \param[in] _address: the start address of register will be written.
    //! \param[in] _numRegistersMinusOne: number of the registers to written - 1
	//! \param[in] chips: which chip register will be written.
    //
    //********************************************************************************************
    void WREGS(uint8_t _address, uint8_t _numRegistersMinusOne, uint8_t chips);
    //********************************************************************************************
    //! \brief Get new conversion data and lead-off detection result in data read continuous mode
    //!
    //! \Must be used after DADY pin go to low(interrupt).
	//! \The data will be storaged in channelData[] and stat[];
    //
    //********************************************************************************************
    void updateChannelData();
    //********************************************************************************************
    //! \brief Print register name for print information.
    //!
	//! \param[in] _address: the address of register will be printed name.
    //
    //********************************************************************************************
    void printRegisterName(uint8_t _address);
    //********************************************************************************************
    //! \brief Print data in HEX format
    //!
	//! \param[in] _data: the data to be printed.
    //
    //********************************************************************************************
    void printHex(uint8_t _data);

    //configuration
    uint8_t ADS_LEADS;                                    //hold leads pf system.
    uint8_t START_PORT, RST_PORT, CS_PORT;                //hold pin information
    uint8_t START_PIN, RST_PIN, CS_PIN[ADS1299_MAX_CHIPS];//hold pin information
    uint16_t stat[ADS1299_MAX_CHIPS];                     //hold lead-off detection result.
    uint8_t regData [24 * ADS1299_MAX_CHIPS];	          //hold all registers value of chips
    long channelData [ADS1299_MAX_CHIPS * ADS_LEADS_PER_CHIP];//hold conversion result.
    bool verbose;		                                  // turn on/off print information
    
    
};

#endif
