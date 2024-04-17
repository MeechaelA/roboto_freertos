/**
 * @file i2c.c
 *
 * Library for I2C on TM4C123GH6PM Microcontroller
 *
 * @author Braedon Giblin <bgiblin@iastate.edu>
 */

#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

void i2c_init() {
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
}

void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt){
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);
    I2CMasterDataPut(I2C0_BASE, reg_addr);
    if (cnt != 1){
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(I2CMasterBusy(I2C0_BASE));
        uint8_t i;
        for(i = 0; i < cnt - 1; i++){
            I2CMasterDataPut(I2C0_BASE, reg_data[i]);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            while(I2CMasterBusy(I2C0_BASE));
        }
        I2CMasterDataPut(I2C0_BASE, reg_data[cnt-1]);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(I2CMasterBusy(I2C0_BASE));
    }
    else{
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        while(I2CMasterBusy(I2C0_BASE));
    }
}

void i2c_single_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt){
}

void i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt){
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);
    I2CMasterDataPut(I2C0_BASE, reg_addr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);
    while(I2CMasterBusy(I2C0_BASE));

    reg_data[0] = I2CMasterDataGet(I2C0_BASE);
    if (cnt != 1){
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        while(I2CMasterBusy(I2C0_BASE));
        uint8_t i;
        for(i = 1; i < cnt - 1; i++){
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            while(I2CMasterBusy(I2C0_BASE));
            reg_data[i] = I2CMasterDataGet(I2C0_BASE);
        }
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C0_BASE));
        reg_data[cnt-1] = I2CMasterDataGet(I2C0_BASE);
    }
    else{
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while(I2CMasterBusy(I2C0_BASE));
    }
}

