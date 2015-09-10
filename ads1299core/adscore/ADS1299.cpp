//
//  ADS1299DIAISY.cpp   ARDUINO LIBRARY FOR COMMUNICATING WITH TWO
//  DAISY-CHAINED ADS1299 BOARDS
//  
//  Created by Conor Russomanno, Luke Travis, and Joel Murphy. Summer, 2013
//
//  Extended by Chip Audette through April 2014
//


#include "gpio.h"
#include "ADS1299.h"
#include "msp430_clock.h"
#include "msp430_spi_a0.h"
#include "msp430_uart.h"
#include "msp430_interrupt.h"
#include "msp430_interrupt_parameters.h"

#define bitRead(data, bit)  ((data & (0x01<<bit))?'1':'0')

void ADS1299::initialize(void (*cb)(void)){
	struct int_param_s int_param;
	int_param.cb = cb;
	int_param.pin = INT_PIN_P21;
	int_param.lp_exit = INT_EXIT_LPM0;
	int_param.active_low = 1;

//	DRDY_PORT = GPIO_PORT_P2;
//	DRDY_PIN = GPIO_PIN1;
	CS_PORT = GPIO_PORT_P6;
	CS_PIN = GPIO_PIN0;
	START_PORT = GPIO_PORT_P4;
	START_PIN = GPIO_PIN0;
	RST_PORT = GPIO_PORT_P3;
	RST_PIN = GPIO_PIN2;
	
//	GPIO_setAsInputPinWithPullUpResistor(DRDY_PORT, DRDY_PIN);
	msp430_reg_int_cb(int_param.cb, int_param.pin, int_param.lp_exit,
			int_param.active_low);

	GPIO_setAsOutputPin(CS_PORT, CS_PIN);
	GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);

	GPIO_setAsOutputPin(START_PORT, START_PIN);
	GPIO_setOutputLowOnPin(START_PORT, START_PIN);

	GPIO_setAsOutputPin(RST_PORT, RST_PIN);
	GPIO_setOutputHighOnPin(RST_PORT, RST_PIN);

	msp430_delay_ms(50);				// recommended power up sequence requiers Tpor (~32mS)
	GPIO_setOutputLowOnPin(RST_PORT, RST_PIN);
	msp430_delay_us(4);	// toggle reset pin
	GPIO_setOutputHighOnPin(RST_PORT, RST_PIN);
	msp430_delay_us(20);	// recommended to wait 18 Tclk before using device (~8uS);
	

    // **** ----- SPI Setup ----- **** //
    
    // Set direction register for SCK and MOSI pin.
    // MISO pin automatically overrides to INPUT.
    // When the SS pin is set as OUTPUT, it can be used as
    // a general purpose output port (it doesn't influence
    // SPI operations).
    
	msp430_spi_a0_init();

    // **** ----- End of SPI Setup ----- **** //
    
}

//System Commands
void ADS1299::WAKEUP() {
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
    msp430_spi_a0_transmitData(_WAKEUP);
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
    msp430_delay_us(3);  		//must wait 4 tCLK cycles before sending another command (Datasheet, pg. 35)
}

void ADS1299::STANDBY() {		// only allowed to send WAKEUP after sending STANDBY
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
    msp430_spi_a0_transmitData(_STANDBY);
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
}

void ADS1299::RESET() {			// reset all the registers to default settings
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
    msp430_spi_a0_transmitData(_RESET);
    msp430_delay_us(12);   	//must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
}

void ADS1299::START() {			//start data conversion 
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
    msp430_spi_a0_transmitData(_START);
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
}

void ADS1299::STOP() {			//stop data conversion
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
    msp430_spi_a0_transmitData(_STOP);
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
}

void ADS1299::RDATAC() {
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
    msp430_spi_a0_transmitData(_RDATAC);
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
	msp430_delay_us(3);
}
void ADS1299::SDATAC() {
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
    msp430_spi_a0_transmitData(_SDATAC);
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);
	msp430_delay_us(3);   //must wait 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}


// Register Read/Write Commands
uint8_t ADS1299::getDeviceID() {			// simple hello world com check
	uint8_t data = RREG(0x00);
	if(verbose){						// verbose otuput
		uint8_t str[] = "Device ID ";
		msp430_uart_write(str, 10);
		printHex(data);	
	}
	return data;
}

uint8_t ADS1299::RREG(uint8_t _address) {		//  reads ONE register at _address
	uint8_t opcode1 = _address + 0x20; 	//  RREG expects 001rrrrr where rrrrr = _address
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN); 				//  open SPI
    msp430_spi_a0_transmitData(opcode1); 					//  opcode1
    msp430_spi_a0_transmitData(0x00); 					//  opcode2
    regData[_address] = msp430_spi_a0_transmitData(0x00);//  update mirror location with returned byte
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN); 			//  close SPI
	if (verbose){						//  verbose output
		printRegisterName(_address);
		printHex(_address);
		uint8_t str[] = ", ";
		msp430_uart_write(str, 2);
		printHex(regData[_address]);
		msp430_uart_write(str, 2);
		uint8_t datbit;
		for(uint8_t j = 0; j<8; j++){
			datbit = bitRead(regData[_address], 7-j);
			msp430_uart_write(&datbit, 1);
			if(j!=7) msp430_uart_write(str, 2);
		}
		str[0] = '\r';
		str[1] = '\n';
		msp430_uart_write(str, 2);
	}
	return regData[_address];			// return requested register value
}

// Read more than one register starting at _address
void ADS1299::RREGS(uint8_t _address, uint8_t _numRegistersMinusOne) {
//	for(byte i = 0; i < 0x17; i++){
//		regData[i] = 0;					//  reset the regData array
//	}
	uint8_t opcode1 = _address + 0x20; 	//  RREG expects 001rrrrr where rrrrr = _address
	regData[0] = 0x00;
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN); 				//  open SPI
    msp430_spi_a0_transmitData(opcode1); 					//  opcode1
    msp430_spi_a0_transmitData(_numRegistersMinusOne);	//  opcode2
    for(uint8_t i = 0; i <= _numRegistersMinusOne; i++){
        regData[_address + i] = msp430_spi_a0_transmitData(0x00); 	//  add register byte to mirror array
		}
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN); 			//  close SPI
	if(verbose){						//  verbose output
		uint8_t str[2];
		uint8_t datbit;
		for(uint8_t i = 0; i<= _numRegistersMinusOne; i++){
			printRegisterName(_address + i);
			printHex(_address + i);
			str[0] = ',';
			str[1] = ' ';
			msp430_uart_write(str, 2);
			printHex(regData[_address + i]);
			msp430_uart_write(str, 2);
			for(uint8_t j = 0; j<8; j++){
				datbit = bitRead(regData[_address + i], 7-j);
				msp430_uart_write(&datbit, 1);
				if(j!=7) msp430_uart_write(str, 2);
			}
			str[0] = '\r';
			str[1] = '\n';
			msp430_uart_write(str, 2);
		}
    }
    
}

void ADS1299::WREG(uint8_t _address, uint8_t _value) {	//  Write ONE register at _address
	uint8_t opcode1 = _address + 0x40; 	//  WREG expects 010rrrrr where rrrrr = _address
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN); 				//  open SPI
    msp430_spi_a0_transmitData(opcode1);					//  Send WREG command & address
    msp430_spi_a0_transmitData(0x00);						//	Send number of registers to read -1
    msp430_spi_a0_transmitData(_value);					//  Write the value to the register
    GPIO_setOutputHighOnPin(CS_PORT, CS_PIN); 			//  close SPI
	regData[_address] = _value;			//  update the mirror array
	if(verbose){						//  verbose output
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

void ADS1299::WREGS(uint8_t _address, uint8_t _numRegistersMinusOne) {
	uint8_t opcode1 = _address + 0x40;		//  WREG expects 010rrrrr where rrrrr = _address
    GPIO_setOutputLowOnPin(CS_PORT, CS_PIN); 				//  open SPI
    msp430_spi_a0_transmitData(opcode1);					//  Send WREG command & address
    msp430_spi_a0_transmitData(_numRegistersMinusOne);	//	Send number of registers to read -1
	for (uint8_t i=_address; i <=(_address + _numRegistersMinusOne); i++){
		msp430_spi_a0_transmitData(regData[i]);			//  Write to the registers
	}	
	GPIO_setOutputHighOnPin(CS_PORT, CS_PIN); 				//  close SPI
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
	uint8_t nchan=8;  //assume 8 channel.  If needed, it automatically changes to 16 automatically in a later block.
	GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);				//  open SPI

	inByte = msp430_spi_a0_transmitData(0x00);//  read 3 byte status register from ADS 1 (1100+LOFF_STATP+LOFF_STATN+GPIO[7:4])
	stat_1 = (stat_1<<8) | inByte;
	inByte = msp430_spi_a0_transmitData(0x00);
	stat_1 = (stat_1<<8) | inByte;
	inByte = msp430_spi_a0_transmitData(0x00);
	stat_1 = (stat_1<<4) | (inByte>>4);
	
	for(uint8_t i = 0; i<8; i++){
		for(uint8_t j=0; j<3; j++){		//  read 24 bits of channel data from 1st ADS in 8 3 byte chunks
			inByte = msp430_spi_a0_transmitData(0x00);
			channelData[i] = (channelData[i]<<8) | inByte;
		}
	}
	GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);				//  close SPI
	
	//reformat the numbers
	for(uint8_t i=0; i<nchan; i++){			// convert 3 byte 2's compliment to 4 byte 2's compliment
		if(channelData[i] & 0x800000){
			channelData[i] |= 0xFF000000;
		}else{
			channelData[i] &= 0x00FFFFFF;
		}
	}
}
	
//read data
void ADS1299::RDATA() {				//  use in Stop Read Continuous mode when DRDY goes low
	uint8_t inByte;
	stat_1 = 0;							//  clear the status registers
//	stat_2 = 0;
	uint8_t nchan = 8;	//assume 8 channel.  If needed, it automatically changes to 16 automatically in a later block.
	GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);				//  open SPI
	msp430_spi_a0_transmitData(_RDATA);
	
	// READ CHANNEL DATA FROM FIRST ADS IN DAISY LINE
	inByte = msp430_spi_a0_transmitData(0x00);//  read 3 byte status register from ADS 1 (1100+LOFF_STATP+LOFF_STATN+GPIO[7:4])
	stat_1 = (stat_1<<8) | inByte;
	inByte = msp430_spi_a0_transmitData(0x00);
	stat_1 = (stat_1<<8) | inByte;
	inByte = msp430_spi_a0_transmitData(0x00);
	stat_1 = (stat_1<<4) | (inByte>>4);
	
	for(uint8_t i = 0; i<8; i++){
		for(uint8_t j=0; j<3; j++){		//  read 24 bits of channel data from 1st ADS in 8 3 byte chunks
			inByte = msp430_spi_a0_transmitData(0x00);
			channelData[i] = (channelData[i]<<8) | inByte;
		}
	}
	for(uint8_t i=0; i<nchan; i++){			// convert 3 byte 2's compliment to 4 byte 2's compliment
		if(channelData[i] & 0x800000){
			channelData[i] |= 0xFF000000;
		}else{
			channelData[i] &= 0x00FFFFFF;
		}
	}
	
    
}



// String-Byte converters for RREG and WREG
void ADS1299::printRegisterName(uint8_t _address) {
    if(_address == ID){
    	uint8_t str0[] = "ID, ";
        msp430_uart_write(str0, 4); //the "F" macro loads the string directly from Flash memory, thereby saving RAM
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


// Used for printing HEX in verbose feedback mode
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

//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//



