/*
 * File : mod_sensors.c
 * Project : e_puck_project
 * Description : Module that acquired and compute signal from e-puck sensors
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "headers/mod_sensors.h"
#include "msgbus/messagebus.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "ch.h"

MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void initSensors(void){
    VL53L0X_start();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
}

uint16_t testSensor(void){
    return VL53L0X_get_dist_mm();
    
}

void getAllProximityValues(int* table){
    proximity_msg_t prox_values;
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
    int i=0;
    for(i=0; i<8;i++){
        table[i] = prox_values.delta[i];
    }
}

int mod_sensor_getValueLightSensor(void){
    
    // WARNING ! TO DO !
}
