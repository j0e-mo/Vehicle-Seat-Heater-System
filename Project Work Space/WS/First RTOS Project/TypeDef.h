/*
 * TypeDef.h
 *
 *  Created on: Sep 15, 2024
 *      Author: Acer
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

typedef enum{
    Heater_OFF, Heater_LOW, Heater_MED, Heater_HIGH
}HeaterIntensityType;

typedef enum{
    Desired_OFF, Desired_LOW = 25, Desired_MED = 30, Desired_HIGH = 35
}DesiredTemperatureType;

typedef struct{
    uint8_t SeatTemperature;
    HeaterIntensityType HeaterState;
    DesiredTemperatureType DesiredTemperature;
}SeatInfoType;



#endif /* TYPEDEF_H_ */
