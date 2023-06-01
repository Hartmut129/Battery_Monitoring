/*
 * eeprom.h
 *
 *  Created on: 20.04.2023
 *      Author: Hartmut
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stdint.h"
#include "i2c.h"

uint8_t address = 0b01010000;

uint8_t setupEeprom(I2C_HandleTypeDef *i2c);


#endif /* INC_EEPROM_H_ */
