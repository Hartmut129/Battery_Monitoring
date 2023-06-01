/*
 * Cell_Measurement.hpp
 *
 *  Created on: Dec 2, 2022
 *      Author: Hartmut
 */

#ifndef INC_CELL_MEASUREMENT_H_
#define INC_CELL_MEASUREMENT_H_

#include "main.h"
#include "adc.h"

 typedef struct Measurement{
	float ext_reference;
	float voltage_cell_1;
	float voltage_cell_2;
	float voltage_cell_3;
	float voltage_cell_4;
	float current;
	float temperature;
	float int_reference;
}ADC_STRUCT;

typedef struct {
	uint16_t result0;
	uint16_t result1;
	uint16_t result2;
	uint16_t result3;
	uint16_t result4;
	uint16_t result5;
	uint16_t result6;
	uint16_t result7;
}ADC_RAW;
//------------------------------------------------------------------------------------------------

#define RESULT_LENGTH 10



extern ADC_STRUCT result[];





//-------------------------------------------------------------------------------------------------

int powerOn(void);
int powerOff(void);
int startMeasurement(ADC_HandleTypeDef*);
void calculateAnalog();
int calibrateZero(void);
int calibrate();
float updateCalibration(const uint16_t* );
void loadAdcCalibrationFromEeprom(void);
void saveAdcCalibrationToEeprom(void);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);


#endif /* INC_CELL_MEASUREMENT_H_ */
