//
//  ADS1299.h
//  Part of the Arduino Library
//  Created by Conor Russomanno, Luke Travis, and Joel Murphy. Summer 2013.
//
//  Modified by Chip Audette through April 2014
//

#ifndef ____ADS1299__
#define ____ADS1299__

#include <stdio.h>
#include <stdint.h>
#include "Definitions.h"


class ADS1299 {
public:
    
    void initialize(void (*cb)(void));
    
    //ADS1299 SPI Command Definitions (Datasheet, p35)
    //System Commands
    void WAKEUP();
    void STANDBY();
    void RESET();
    void START();
    void STOP();
    
    //Data Read Commands
    void RDATAC();
    void SDATAC();
    void RDATA();
    
    //Register Read/Write Commands
    uint8_t getDeviceID();
    uint8_t RREG(uint8_t _address);
    void RREGS(uint8_t _address, uint8_t _numRegistersMinusOne);
    void printRegisterName(uint8_t _address);
    void WREG(uint8_t _address, uint8_t _value);
    void WREGS(uint8_t _address, uint8_t _numRegistersMinusOne);
    void printHex(uint8_t _data);
    void updateChannelData();
    

    //configuration
    uint8_t CS_PORT, START_PORT, RST_PORT; 		// pin numbers for DRDY and CS
    uint8_t CS_PIN, START_PIN, RST_PIN; 		// pin numbers for DRDY and CS
    uint16_t stat_1, stat_2;    // used to hold the status register for boards 1 and 2
    uint8_t regData [24];	// array is used to mirror register data
    long channelData [16];	// array used when reading channel data board 1+2
    bool verbose;		// turn on/off Serial feedback
    
    
};

#endif
