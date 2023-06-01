/*
 * Cell_Measurement.cpp
 *
 *  Created on: Dec 2, 2022
 *      Author: Hartmut
 */

#include "cell_measurement.h"
#include <stdint.h>
#include "usb_vcp.h"
#include "string.h"

#define STARTUP_TIME 20
#define SHUTDOWN_TIME 10
								// Buffer wird als 16bit bzw half word angelegt
#define CONVERSION_LENGTH ((sizeof(ADC_RAW)/2)  * 2)   // *2 ermöglicht einen geteilten buffer wo eine Hälfte geschrieben und die andere gleichzeitig gelesen wird
                           // Historie für Mittelwert

#define V_REF 3.3              // VDDA used as reference since 48pin versions do not have a reference input
#define VCAL1 1.2               // Value of an external Voltage Reference this is used to calibrate the ADC
#define VCAL2 3.0               // Value of an external Voltage Reference this is used to calibrate the ADC
#define ADC_RESOLUTION 4096    //12Bit Resolution
#define CURRENT_CONVERSION  1.0  // Factor used to calculate the actual current from the voltage delivered by the shunt_amplifier
#define AVG_SLOPE 4.3 	//mV\°C
#define V_25 1.43		//V




#define false 0
#define true 1



//#######################################################################################

float toVolt(const uint16_t* adcresult);
float toTemperature(const uint16_t* adc_result);
inline float updateCalibration(const uint16_t* calibration_value);
//#######################################################################################

 uint16_t raw_buffer[CONVERSION_LENGTH] = {0};
 uint16_t copy_buffer[CONVERSION_LENGTH/2] = {0}; // wird mit der fertigen Hälfte des raw_buffer beschrieben
 ADC_STRUCT result[RESULT_LENGTH] = {0};
 uint8_t buffer_half = 0;
 uint8_t buffer_full = 0;
const uint16_t half_complete_offset = CONVERSION_LENGTH/2;


uint16_t adc_offset_error = 0;
double adc_linearity_error = 1.0;
double calibration_factor = 1.0;


volatile uint8_t data_valid = false;
//#######################################################################################


/// @brief enables Power for the analog circuit
///
/// @return returns 0 if successful, does not return if failed
int powerOn(){

	HAL_GPIO_WritePin(FET_POWER_GPIO_Port, FET_POWER_Pin, HIGH);
	HAL_Delay(STARTUP_TIME);
	HAL_GPIO_WritePin(FET_SENSE_GPIO_Port, FET_SENSE_Pin, HIGH);
	HAL_Delay(STARTUP_TIME);

	return HAL_OK;
}
//--------------------------------------------------------------------

/// @brief stops ADC conversion and turns the analog circuit power supply off
///
/// @return  0 if successful
int powerOff(){
	int hal_status = HAL_OK;
	hal_status = HAL_ADC_Stop_DMA(&hadc1);

	HAL_GPIO_WritePin(FET_SENSE_GPIO_Port, FET_SENSE_Pin, LOW);
	HAL_Delay(SHUTDOWN_TIME);
	HAL_GPIO_WritePin(FET_POWER_GPIO_Port, FET_POWER_Pin, LOW);
	HAL_Delay(SHUTDOWN_TIME);

	return hal_status;
}
//--------------------------------------------------------------------

/// @brief Starts the ADC using DMA transfer
///
/// @return HAL Status
int startMeasurement(ADC_HandleTypeDef *adc){

	return HAL_ADC_Start_DMA(adc, (uint32_t*)raw_buffer, (uint32_t)CONVERSION_LENGTH);
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/// @brief
/// Ich bin dooof
///
void calculateAnalog(){

ADC_STRUCT temp = {0};

#ifdef DEBUG_ADC
static uint32_t loopcounter = 0;
char text[30];

		if(buffer_half == 1 && buffer_full == 0){
			memset(text,0,sizeof(text));
			strcpy(text,"DMA HALF\r\n");
			usb_vcp_send(text, sizeof(text));
		}else if(buffer_full == 1 && buffer_half == 0){
			memset(text,0,sizeof(text));
			strcpy(text,"DMA FULL\r\n");
			usb_vcp_send(text, sizeof(text));
		}else{

			loopcounter++;
			memset(text,0,sizeof(text));
			itoa(loopcounter,text,10);
			usb_vcp_send((uint8_t*)text, sizeof(text));
			memset(text,0,sizeof(text));
			strcpy(text,"DMA Error\r\n");
			usb_vcp_send((uint8_t*)text, sizeof(text));
		}
#endif
	updateCalibration(copy_buffer +7);
	temp.ext_reference = toVolt(copy_buffer);
	temp.voltage_cell_1 = toVolt(copy_buffer +1);
	temp.voltage_cell_2 = toVolt(copy_buffer +2);
	temp.voltage_cell_3 = toVolt(copy_buffer +3);
	temp.voltage_cell_4 = toVolt(copy_buffer +4);
	temp.current = CURRENT_CONVERSION * toVolt(copy_buffer +5);
	temp.temperature = toTemperature(copy_buffer +6);
	temp.int_reference = toVolt(copy_buffer +7);

	// shift results

	for(uint8_t i = 0; i < (RESULT_LENGTH -1); i++){
		result[i] = result[i + 1];
	}
	result[RESULT_LENGTH -1] = temp;

}


//--------------------------------------------------------------------

/// @brief translates ADC value to measured Voltage
///
/// @param adcresult  pointer to the current ADC  measurement
/// @param calibrationfactor A factor to correct offset error
/// @return measured Voltage
inline float toVolt(const uint16_t* adcresult){
	float temp = *adcresult * V_REF / ADC_RESOLUTION;
	temp *= calibration_factor;
	return temp;
}
//--------------------------------------------------------------------


//--------------------------------------------------------------------

/// @brief Calculates Temperature in °C for the internal Sensor
///
/// @param address of adc result
/// @return Temperature in °Celsius
float toTemperature(const uint16_t* adc_result){
	float temperature = 0;
	//V_25 in Volt   AVG_SLOPE in mV pro °C
	temperature  = ((V_25 - toVolt(adc_result) ) / AVG_SLOPE*1000) + 25; //(in °C)
	return temperature;
}

//--------------------------------------------------------------------

/// @brief calculates a calibration factor  from an external voltage reference
///		   since the ADC uses VDDA as Reference this should be called regularly
///
/// @param calibration_value
/// @return factor that describes difference between measured and actual reference
inline float updateCalibration(const uint16_t* calibration_value){
	double result = VCAL1/(*calibration_value * (V_REF / ADC_RESOLUTION));	 //3.0V Reference is more precise
	calibration_factor = result;
	return (float)result;
}
//--------------------------------------------------------------------



void loadAdcCalibrationFromEeprom(void){

}
//--------------------------------------------------------------------

void saveAdcCalibrationToEeprom(void){

}

//--------------------------------------------------------------------
//	ADC buffer Status: der Buffer enthält 2 Blöcke. Einer, der aktuell mit ADC Ergebnissen gefüllt wird
//	und einer, der die Ergebnisse des letzten Konversionszykluses enthält.
//	Da nicht bekannt ist welchen ADC Kanal der DMA gerade übertragen hat und wann der nächste übertragen wird,
//	dürfen nur Daten aus dem gerade nicht aktiven Teil des Buffers zur Weiterverarbeitung gelesen werden.
//	Da der DMA zirkulär arbeitet, schaltet er automatisch auf den anderen Bufferteil um sobalt er einen Block fertig beschrieben hat.
//	es muss also vor jedem Zugriff getestet werden aus welchem Teil die Daten gelesen werden dürfen.
//--------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	  for(uint8_t i = 0; i < CONVERSION_LENGTH/2; i++){
		  copy_buffer[i] = raw_buffer[i + half_complete_offset];
	  }
	  buffer_full = 1;
	  buffer_half = 0;

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){

	  for(uint8_t i = 0; i < CONVERSION_LENGTH/2; i++){
		  copy_buffer[i] = raw_buffer[i];
	  }
	  buffer_full = 0;
	  buffer_half = 1;

}



