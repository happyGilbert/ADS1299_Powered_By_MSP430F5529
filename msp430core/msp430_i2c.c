/*
 * msp430_i2c.c
 *
 *  Created on: 2015Äê9ÔÂ4ÈÕ
 *      Author: jfanl
 */
#include "gpio.h"
#include "usci_b_i2c.h"
#include "msp430_i2c.h"
#include "msp430_clock.h"
#include "inc/hw_memmap.h"

#define I2C_TIMEOUT  (8000)
#define I2C_TIMEOUT_MS  (2500)

typedef enum {
    STATE_WAITING,
    STATE_READING,
    STATE_WRITING
} msp430_i2c_state;

typedef struct {
    volatile msp430_i2c_state state;
    /* First slave register. */
    unsigned char slave_reg;
    unsigned char *data;
    unsigned short length;
    unsigned char enabled;
} msp430_i2c_info;

static msp430_i2c_info i2c = {
    .enabled = 0
};

void msp430_i2c_init(void)
{
	unsigned long _smclk;
    if (i2c.enabled)
        return;
    /**Configure Pins for I2C
     * Set P3.0 and P3.1 as Secondary Module Function Input.
     * Select Port 3
     * Set Pin 0, 1 to input Secondary Module Function, (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
     * */
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P3,
            GPIO_PIN0 + GPIO_PIN1
            );
    _smclk = msp430_get_smclk_freq();
    USCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = USCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = _smclk;
    param.dataRate = USCI_B_I2C_SET_DATA_RATE_400KBPS;
    USCI_B_I2C_initMaster(USCI_B0_BASE, &param);
    //Enable I2C Module to start operations
    USCI_B_I2C_enable(USCI_B0_BASE);

    USCI_B_I2C_clearInterrupt(USCI_B0_BASE,
            USCI_B_I2C_TRANSMIT_INTERRUPT +
            USCI_B_I2C_RECEIVE_INTERRUPT +
            USCI_B_I2C_NAK_INTERRUPT
            );
    //Enable master Receive interrupt
    USCI_B_I2C_enableInterrupt(USCI_B0_BASE,
            USCI_B_I2C_TRANSMIT_INTERRUPT +
            USCI_B_I2C_RECEIVE_INTERRUPT +
            USCI_B_I2C_NAK_INTERRUPT
            );

    /* Initialize struct. */
    i2c.state = STATE_WAITING;
    i2c.slave_reg = 0;
    i2c.data = 0;
    i2c.length = 0;
    i2c.enabled = 1;
}

void msp430_i2c_disable(void)
{
    if (!i2c.enabled)
        return;
    USCI_B_I2C_disable(USCI_B0_BASE);
    GPIO_setAsOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN0 + GPIO_PIN0
            );
    GPIO_setOutputHighOnPin(
            GPIO_PORT_P3,
            GPIO_PIN0 + GPIO_PIN0
            );
    i2c.enabled = 0;
    return;
}

int8_t msp430_i2c_write(unsigned char slave_addr,
        unsigned char reg_addr,
        unsigned char length,
        unsigned char const *data)
{
	unsigned long start, cur;
    if (!i2c.enabled)
        return -1;
    if (!length)
        return 0;

    /* Populate struct. */
    i2c.state = STATE_WRITING;
    i2c.slave_reg = reg_addr;
    i2c.data = (unsigned char*)data;
    i2c.length = length;

    //Specify slave address
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE,
            slave_addr
            );

    //Set Master in transmit mode
    USCI_B_I2C_setMode(USCI_B0_BASE,
            USCI_B_I2C_TRANSMIT_MODE
            );

    while(USCI_B_I2C_SENDING_STOP ==
    		USCI_B_I2C_masterIsStopSent(USCI_B0_BASE));

    if(USCI_B_I2C_masterSendMultiByteStartWithTimeout(USCI_B0_BASE,
 		   i2c.slave_reg,
 		   I2C_TIMEOUT) == STATUS_FAIL)

    {
    	USCI_B_I2C_masterSendMultiByteStop(USCI_B0_BASE);
        i2c.state = STATE_WAITING;
        msp430_i2c_disable();
        msp430_delay_ms(1);
        GPIO_setOutputLowOnPin(
                GPIO_PORT_P3,
                GPIO_PIN0 + GPIO_PIN0
                );
        msp430_delay_ms(1);
        GPIO_setOutputHighOnPin(
                GPIO_PORT_P3,
                GPIO_PIN0 + GPIO_PIN0
                );
        msp430_i2c_init();
        return -1;
    }

    msp430_get_clock_ms(&start);
    while (i2c.state != STATE_WAITING) {
        __bis_SR_register(LPM0_bits + GIE);
        msp430_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS)) {
        	USCI_B_I2C_masterSendMultiByteStop(USCI_B0_BASE);
            i2c.state = STATE_WAITING;
            msp430_i2c_disable();
            msp430_delay_ms(1);
            GPIO_setOutputLowOnPin(
                    GPIO_PORT_P3,
                    GPIO_PIN0 + GPIO_PIN0
                    );
            msp430_delay_ms(1);
            GPIO_setOutputHighOnPin(
                    GPIO_PORT_P3,
                    GPIO_PIN0 + GPIO_PIN0
                    );
            msp430_i2c_init();
            return -1;
        }
    }
    return 0;
}

int8_t msp430_i2c_read(unsigned char slave_addr,
        unsigned char reg_addr,
        unsigned char length,
        unsigned char *data)
{
    unsigned long start, cur;
    if (!i2c.enabled)
        return -1;
    if (!length)
        return 0;

    /* Populate struct. */
    i2c.state = STATE_READING;
    i2c.slave_reg = reg_addr;
    i2c.data = data;
    i2c.length = length;

    //Specify slave address
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE,
            slave_addr
            );

    //Set Master in transmit mode
    USCI_B_I2C_setMode(USCI_B0_BASE,
            USCI_B_I2C_TRANSMIT_MODE
            );

    while(USCI_B_I2C_SENDING_STOP ==
    		USCI_B_I2C_masterIsStopSent(USCI_B0_BASE));

    if(USCI_B_I2C_masterSendMultiByteStartWithTimeout
    		  (USCI_B0_BASE,i2c.slave_reg,
    		   I2C_TIMEOUT) == STATUS_FAIL)
    {
    	USCI_B_I2C_masterSendMultiByteStop(USCI_B0_BASE);
        i2c.state = STATE_WAITING;
        msp430_i2c_disable();
        msp430_delay_ms(1);
        GPIO_setOutputLowOnPin(
                GPIO_PORT_P3,
                GPIO_PIN0 + GPIO_PIN0
                );
        msp430_delay_ms(1);
        GPIO_setOutputHighOnPin(
                GPIO_PORT_P3,
                GPIO_PIN0 + GPIO_PIN0
                );
        msp430_i2c_init();
        return -1;
    }

    msp430_get_clock_ms(&start);
    while (i2c.state != STATE_WAITING) {
        __bis_SR_register(LPM0_bits + GIE);
        msp430_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS)) {
        	USCI_B_I2C_masterReceiveMultiByteStop(USCI_B0_BASE);
            i2c.state = STATE_WAITING;
            msp430_i2c_disable();
            msp430_delay_ms(1);
            GPIO_setOutputLowOnPin(
                    GPIO_PORT_P3,
                    GPIO_PIN0 + GPIO_PIN0
                    );
            msp430_delay_ms(1);
            GPIO_setOutputHighOnPin(
                    GPIO_PORT_P3,
                    GPIO_PIN0 + GPIO_PIN0
                    );
            msp430_i2c_init();
            return -1;
        }
    }
    return 0;
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void)
{
    switch(__even_in_range(UCB0IV,0x0C))
    {
        case 0x04:     /* NAK interrupt. */
        {
            i2c.state = STATE_WAITING;
            USCI_B_I2C_masterSendMultiByteStop(USCI_B0_BASE);
            break;
        }
        case 0x0A:    /* RX interrupt. */
        {
            if (--i2c.length) {
                /* If only one byte left, prepare stop signal. */
                if (i2c.length == 1)
                	USCI_B_I2C_masterReceiveMultiByteStop(USCI_B0_BASE);
            } else
                i2c.state = STATE_WAITING;
            /* Read RXBUF last because we don't want to release SCL until we're
             * sure we're ready.
             */
            *i2c.data++ = USCI_B_I2C_masterReceiveMultiByteNext(USCI_B0_BASE);
            break;
        }
        case 0x0C:    /* TX interrupt. */
        {
            switch (i2c.state) {
            case STATE_WRITING:
            {
            	if (i2c.length) {
            		/* Send next byte, increment pointer. */
            		USCI_B_I2C_masterSendMultiByteNext(USCI_B0_BASE,
            					                        i2c.data[0]
            					                        );
            		i2c.data++;
            		i2c.length--;
            	} else {
            		USCI_B_I2C_masterSendMultiByteStop(USCI_B0_BASE);
            		i2c.state = STATE_WAITING;
            	}
                break;
            }
            case STATE_READING:
            {
            	/* Repeated start, switch to RX mode. */
            	USCI_B_I2C_masterReceiveMultiByteStart(USCI_B0_BASE);
            	/* If single byte, prepare stop signal immediately. */
            	if (i2c.length == 1) {
            		/* Well, not IMMEDIATELY. First we need to make sure
            		 * the start signal got sent.
            		 * */
            		while (USCI_B_I2C_masterIsStartSent(USCI_B0_BASE) ==
            			   USCI_B_I2C_SENDING_START);
            		USCI_B_I2C_masterReceiveMultiByteStop(USCI_B0_BASE);
            	}
                break;
            }
            case STATE_WAITING:
            default:
                break;
            }
            break;
        }
        case 0:     /* No interrupt. */
        case 2:     /* Arbitration lost interrupt. */
        case 6:     /* Start condition interrupt. */
        case 8:     /* Stop condition interrupt. */
        default:
            break;
    }
    __bic_SR_register_on_exit(LPM0_bits);
}

