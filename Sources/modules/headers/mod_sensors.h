/*
 * File : mod_sensors.h
 * Project : e_puck_project
 * Description : Module that acquired and compute signal from e-puck sensors
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_SENSORS_
#define _MOD_SENSORS_

#include <inttypes.h>

void initSensors(void);

uint16_t testSensor(void);

int mod_sensor_getValueLightSensor(void);

void getAllProximityValues(int* table);

#endif
