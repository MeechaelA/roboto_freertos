

#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <inc/tm4c123gh6pm.h>

/**
 * @brief Initialize I2C1 module.
 *
 * Initializes I2C1 module as a master at standard speed.
 */
void i2c_init();
void i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);


#endif I2C_H_
