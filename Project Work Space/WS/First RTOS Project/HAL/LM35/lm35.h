

#ifndef HAL_LM35_LM35_H_
#define HAL_LM35_LM35_H_

#include <stdint.h>

#define DriverSensor_MAX_VALUE   4096
#define PassengerSensor_MAX_VALUE   4096

void DriverSensor_Init(void);
uint8_t GetDriverSensorReading(void);

void PassengerSensor_Init(void);
uint8_t GetPassengerSensorReading(void);


#endif /* HAL_LM35_LM35_H_*/
