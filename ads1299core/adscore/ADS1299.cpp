/*
 * ADS1299.cpp with msp430f5529
 *
 *  Created on: 2015Äê9ÔÂ8ÈÕ
 *      Author: jfanl
 */
#include "gpio.h"
#include "ADS1299.h"
#include "msp430_uart.h"
#include "msp430_clock.h"
#include "msp430_spi_a0.h"
#include "msp430_interrupt.h"
#include "msp430_interrupt_parameters.h"

#define bitRead(data, bit)  ((data & (0x01<<bit))?'1':'0')


//********************************************************************************************
//
//! \brief Initializes ADS1299.
//!
//! pin map: MSP430F5529  <--->  ADS1299
//!          P2.7         <--->  SCLK --|
//!          P3.3         <--->  SIMO   |---USCI A0
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
void ADS1299::initialize(void (*cb)(void), uint8_t ads_leads){
	struct int_param_s int_param;
	int_param.cb = cb;
	int_param.pin = INT_PIN_P21;      //p2.1 interrput for DRDY pin
	int_param.lp_exit = INT_EXIT_LPM0;//after return of interrupt, program will exit lpm0 mode.
	int_param.active_low = 1;         //low-active

	ADS_LEADS = ads_leads;

//	DRDY_PORT = GPIO_PORT_P2;
//	DRDY_PIN = GPIO_PIN1;

	START_PORT = GPIO_PORT_P4;
	START_PIN = GPIO_PIN0;
	RST_PORT = GPIO_PORT_P3;
	RST_PIN = GPIO_PIN2;
	CS_PORT = GPIO_PORT_P6;
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
		CS_PIN[chips] = (0x0001 << chips);
	}                                 //initialize CS pin with BIT0, BIT1,......

	
//	GPIO_setAsInputPinWithPullUpResistor(DRDY_PORT, DRDY_PIN);
	msp430_reg_int_cb(int_param.cb, int_param.pin, int_param.lp_exit,
			int_param.active_low);   //register interrupt function and parameters

	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){  //initialize pin state.
		GPIO_setAsOutputPin(CS_PORT, CS_PIN[chips]);
		GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	}

	GPIO_setAsOutputPin(START_PORT, START_PIN);
	GPIO_setOutputLowOnPin(START_PORT, START_PIN);

	GPIO_setAsOutputPin(RST_PORT, RST_PIN);
	GPIO_setOutputHighOnPin(RST_PORT, RST_PIN);

	msp430_delay_ms(50);				// recommended power up sequence requiers Tpor (~32mS)
	GPIO_setOutputLowOnPin(RST_PORT, RST_PIN);  //reset ads1299
	msp430_delay_us(4);
	GPIO_setOutputHighOnPin(RST_PORT, RST_PIN);
	msp430_delay_us(20);	         // recommended to wait 18 Tclk before using device (~8uS)
    
	msp430_spi_a0_init();            //initialize SPI with 4MHZ SCLK, MSB, UCCKPH:0, UCCKPl:0
    
}

/*System Commands, command will send to all chip in multi-chip ads1299*/
void ADS1299::WAKEUP() {            //wake up chips which stay in standby mode
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
	    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
	    msp430_spi_a0_transmitData(_WAKEUP);
	    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	    msp430_delay_us(3);  		//must wait 4 tCLK cycles before sending another command (Datasheet, pg. 35)
	}
}

void ADS1299::STANDBY() {		   // get into standby mode
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
	    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
	    msp430_spi_a0_transmitData(_STANDBY);
	    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	}
}

void ADS1299::RESET() {			   // reset chip and all the registers to default settings
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
	    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
	    msp430_spi_a0_transmitData(_RESET);
	    msp430_delay_us(12);   	//must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
	    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	}
}

void ADS1299::START() {			//start data conversion 
//    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
//    msp430_spi_a0_transmitData(_START);
//    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
	GPIO_setOutputHighOnPin(START_PORT, START_PIN);
}  //multi-chips must use start pin to synchronous start conversion, not use start command

void ADS1299::STOP() {			//stop data conversion
//    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
//    msp430_spi_a0_transmitData(_STOP);
//    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
	GPIO_setOutputLowOnPin(START_PORT, START_PIN);
}  //multi-chips must use start pin to synchronous stop conversion, not use stop command

void ADS1299::RDATAC() {       //enable read data continuous mode
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
	    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
	    msp430_spi_a0_transmitData(_RDATAC);
	    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
		msp430_delay_us(3);   //must wait 4 tCLK cycles after executing this command (Datasheet, pg. 37)
	}
}
void ADS1299::SDATAC() {      //stop read data continuous mode
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
	    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
	    msp430_spi_a0_transmitData(_SDATAC);
	    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
		msp430_delay_us(3);   //must wait 4 tCLK cycles after executing this command (Datasheet, pg. 37)
	}
}


/*Register Read/Write Commands for multi-chips system.*/
uint8_t ADS1299::getDeviceID(uint8_t chips) {// get chip's ID for checking chip and communication function.
	uint8_t data = RREG(0x00, chips);
	if(verbose){						     // verbose to print information
		uint8_t str[] = "Device ID ";
		msp430_uart_write(str, 10);
		printHex(data);	
	}
	return data;
}

uint8_t ADS1299::RREG(uint8_t _address, uint8_t chips) {//  reads one register value
	uint8_t opcode1 = _address + 0x20; 	                //  opcode1 = 0x20 + register's address
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
    msp430_spi_a0_transmitData(opcode1);
    msp430_spi_a0_transmitData(0x00); 					//  opcode2 = number of the registers to - 1
    regData[_address + chips * 24] = msp430_spi_a0_transmitData(0x00);//  get register's value
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	if (verbose){
		printRegisterName(_address);
		printHex(_address);
		uint8_t str[] = ", ";
		msp430_uart_write(str, 2);
		printHex(regData[_address + chips * 24]);
		msp430_uart_write(str, 2);
		uint8_t datbit;
		for(uint8_t j = 0; j<8; j++){
			datbit = bitRead(regData[_address + chips * 24], 7-j);
			msp430_uart_write(&datbit, 1);
			if(j!=7) msp430_uart_write(str, 2);
		}
		str[0] = '\r';
		str[1] = '\n';
		msp430_uart_write(str, 2);
	}
	return regData[_address + chips * 24];			  // return register value
}

// Read more than one register starting at _address
void ADS1299::RREGS(uint8_t _address, uint8_t _numRegistersMinusOne, uint8_t chips) {
	uint8_t opcode1 = _address + 0x20; 	              //  opcode1 = 0x20 + register's address
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
    msp430_spi_a0_transmitData(opcode1);
    msp430_spi_a0_transmitData(_numRegistersMinusOne);//  opcode2 = number of the registers to read - 1
    for(uint8_t i = 0; i <= _numRegistersMinusOne; i++){
        regData[_address + i + chips * 24] = msp430_spi_a0_transmitData(0x00); 	//  get register's value.
		}
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	if(verbose){
		uint8_t str[2];
		uint8_t datbit;
		for(uint8_t i = 0; i<= _numRegistersMinusOne; i++){
			printRegisterName(_address + i);
			printHex(_address + i);
			str[0] = ',';
			str[1] = ' ';
			msp430_uart_write(str, 2);
			printHex(regData[_address + i + chips * 24]);
			msp430_uart_write(str, 2);
			for(uint8_t j = 0; j<8; j++){
				datbit = bitRead(regData[_address + i + chips * 24], 7-j);
				msp430_uart_write(&datbit, 1);
				if(j!=7) msp430_uart_write(str, 2);
			}
			str[0] = '\r';
			str[1] = '\n';
			msp430_uart_write(str, 2);
		}
    }
    
}

void ADS1299::WREG(uint8_t _address, uint8_t _value, uint8_t chips) {//  Write ONE register at _address
	uint8_t opcode1 = _address + 0x40; 	                     //  opcode1 = 0x40 + register's address
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
    msp430_spi_a0_transmitData(opcode1);
    msp430_spi_a0_transmitData(0x00);						 //  opcode2 = number of the registers to write - 1
    msp430_spi_a0_transmitData(_value);					     //  Write the value to the register
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	regData[_address + chips * 24] = _value;
	if(verbose){
		uint8_t str[] = "Register ";
		msp430_uart_write(str, 9);
		printHex(_address);
		uint8_t str1[] = " modified.";
		msp430_uart_write(str1, 10);
		str[0] = '\r';
		str[1] = '\n';
		msp430_uart_write(str, 2);
	}
}

void ADS1299::WREGS(uint8_t _address, uint8_t _numRegistersMinusOne, uint8_t chips) {
	uint8_t opcode1 = _address + 0x40;		                 //  opcode1 = 0x40 + register's address
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
    msp430_spi_a0_transmitData(opcode1);
    msp430_spi_a0_transmitData(_numRegistersMinusOne);	     //  opcode2 = number of the registers to write - 1
	for (uint8_t i=_address; i <=(_address + _numRegistersMinusOne); i++){
		msp430_spi_a0_transmitData(regData[i + chips * 24]); //  Write to the registers
	}	
	GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	if(verbose){
		uint8_t str[] = "Register ";
		msp430_uart_write(str, 9);
		printHex(_address);
		str[0] = ' ';
		str[1] = 't';
		str[2] = 'o';
		str[3] = ' ';
		msp430_uart_write(str, 4);
		printHex(_address + _numRegistersMinusOne);
		uint8_t str1[] = " modified.";
		msp430_uart_write(str1, 10);
		str[0] = '\r';
		str[1] = '\n';
		msp430_uart_write(str, 2);
	}
}

void ADS1299::updateChannelData(){
	uint8_t inByte;
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
		GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);

		inByte = msp430_spi_a0_transmitData(0x00);//  read 3 byte status register(1100+LOFF_STATP+LOFF_STATN+GPIO[7:4]) to get lead-off detection result
		stat[chips] = (stat[chips]<<8) | inByte;
		inByte = msp430_spi_a0_transmitData(0x00);
		stat[chips] = (stat[chips]<<8) | inByte;
		inByte = msp430_spi_a0_transmitData(0x00);
		stat[chips] = (stat[chips]<<4) | (inByte>>4);

		for(uint8_t i = 0; i<8; i++){
			for(uint8_t j=0; j<3; j++){		//  read 24 bits channel data
				inByte = msp430_spi_a0_transmitData(0x00);
				channelData[i + chips * ADS_LEADS_PER_CHIP] = (channelData[i + chips * ADS_LEADS_PER_CHIP]<<8) | inByte;
			}
		}
		GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	}
	
	//reformat data format to long
	for(uint8_t i=0; i < (ADS_LEADS * ADS_LEADS_PER_CHIP); i++){
		if(channelData[i] & 0x800000){// convert 3 byte 2's compliment to 4 byte 2's compliment
			channelData[i] |= 0xFF000000;
		}else{
			channelData[i] &= 0x00FFFFFF;
		}
	}
}
	
//read data by command: single-shot conversion
void ADS1299::RDATA() {
	uint8_t inByte;
	for(uint8_t chips = 0; chips < ADS_LEADS; chips++){
		GPIO_setOutputLowOnPin(CS_PORT, CS_PIN[chips]);
		msp430_spi_a0_transmitData(_RDATA);
		inByte = msp430_spi_a0_transmitData(0x00);//  read 3 byte status register(1100+LOFF_STATP+LOFF_STATN+GPIO[7:4]) to get lead-off detection result
		stat[chips] = (stat[chips]<<8) | inByte;
		inByte = msp430_spi_a0_transmitData(0x00);
		stat[chips] = (stat[chips]<<8) | inByte;
		inByte = msp430_spi_a0_transmitData(0x00);
		stat[chips] = (stat[chips]<<4) | (inByte>>4);

		for(uint8_t i = 0; i<8; i++){
			for(uint8_t j=0; j<3; j++){		//  read 24 bits channel data
				inByte = msp430_spi_a0_transmitData(0x00);
				channelData[i + chips * ADS_LEADS_PER_CHIP] = (channelData[i + chips * ADS_LEADS_PER_CHIP]<<8) | inByte;
			}
		}
		GPIO_setOutputHighOnPin(CS_PORT, CS_PIN[chips]);
	}

	//reformat data format to long
	for(uint8_t i=0; i < (ADS_LEADS * ADS_LEADS_PER_CHIP); i++){
		if(channelData[i] & 0x800000){// convert 3 byte 2's compliment to 4 byte 2's compliment
			channelData[i] |= 0xFF000000;
		}else{
			channelData[i] &= 0x00FFFFFF;
		}
	}
}



// print register name for register read and write to print information
void ADS1299::printRegisterName(uint8_t _address) {
    if(_address == ID){
    	uint8_t str0[] = "ID, ";
        msp430_uart_write(str0, 4);
    }
    else if(_address == CONFIG1){
    	uint8_t str1[] = "CONFIG1, ";
        msp430_uart_write(str1, 9);
    }
    else if(_address == CONFIG2){
    	uint8_t str2[] = "CONFIG2, ";
        msp430_uart_write(str2, 9);
    }
    else if(_address == CONFIG3){
    	uint8_t str3[] = "CONFIG3, ";
        msp430_uart_write(str3, 9);
    }
    else if(_address == LOFF){
    	uint8_t str4[] = "LOFF, ";
        msp430_uart_write(str4, 6);
    }
    else if(_address == CH1SET){
    	uint8_t str5[] = "CH1SET, ";
        msp430_uart_write(str5, 8);
    }
    else if(_address == CH2SET){
    	uint8_t str6[] = "CH2SET, ";
        msp430_uart_write(str6, 8);
    }
    else if(_address == CH3SET){
    	uint8_t str7[] = "CH3SET, ";
        msp430_uart_write(str7, 8);
    }
    else if(_address == CH4SET){
    	uint8_t str8[] = "CH4SET, ";
        msp430_uart_write(str8, 8);
    }
    else if(_address == CH5SET){
    	uint8_t str9[] = "CH5SET, ";
        msp430_uart_write(str9, 8);
    }
    else if(_address == CH6SET){
    	uint8_t stra[] = "CH6SET, ";
        msp430_uart_write(stra, 8);
    }
    else if(_address == CH7SET){
    	uint8_t strb[] = "CH7SET, ";
        msp430_uart_write(strb, 8);
    }
    else if(_address == CH8SET){
    	uint8_t strc[] = "CH8SET, ";
        msp430_uart_write(strc, 8);
    }
    else if(_address == BIAS_SENSP){
    	uint8_t strd[] = "BIAS_SENSP, ";
        msp430_uart_write(strd, 12);
    }
    else if(_address == BIAS_SENSN){
    	uint8_t stre[] = "BIAS_SENSN, ";
        msp430_uart_write(stre, 12);
    }
    else if(_address == LOFF_SENSP){
    	uint8_t strf[] = "LOFF_SENSP, ";
        msp430_uart_write(strf, 12);
    }
    else if(_address == LOFF_SENSN){
    	uint8_t strg[] = "LOFF_SENSN, ";
        msp430_uart_write(strg, 12);
    }
    else if(_address == LOFF_FLIP){
    	uint8_t strh[] = "LOFF_FLIP, ";
        msp430_uart_write(strh, 11);
    }
    else if(_address == LOFF_STATP){
    	uint8_t stri[] = "LOFF_STATP, ";
        msp430_uart_write(stri, 12);
    }
    else if(_address == LOFF_STATN){
    	uint8_t strj[] = "LOFF_STATN, ";
        msp430_uart_write(strj, 12);
    }
    else if(_address == GPIO){
    	uint8_t strk[] = "GPIO, ";
        msp430_uart_write(strk, 6);
    }
    else if(_address == MISC1){
    	uint8_t strl[] = "MISC1, ";
        msp430_uart_write(strl, 7);
    }
    else if(_address == MISC2){
    	uint8_t strm[] = "MISC2, ";
        msp430_uart_write(strm, 7);
    }
    else if(_address == CONFIG4){
    	uint8_t strn[] = "CONFIG4, ";
        msp430_uart_write(strn, 9);
    }
}


// print data with HEX format
void ADS1299::printHex(uint8_t _data){
	uint8_t str[] = "0x";
	msp430_uart_write(str, 2);
	if(_data >= 0xa0)
	{
		str[0] = 'A' + (((_data & 0xf0) - 0xa0)>>4);
	}
	else
	{
		str[0] = '0' + ((_data & 0xf0)>>4);
	}
	msp430_uart_write(str, 1);
	if((_data & 0x0f) >= 0x0a)
	{
		str[0] = 'A' + ((_data & 0x0f) - 0x0a);
	}
	else
	{
		str[0] = '0' + (_data & 0x0f);
	}
	msp430_uart_write(str, 1);
}




