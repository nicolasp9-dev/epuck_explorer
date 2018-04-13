/*
 * File : mod_sensors.c
 * Project : e_puck_project
 * Description : Module that acquired and compute signal from e-puck sensors
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "headers/mod_sensors.h"

// Standard headers
#include <inttypes.h>

// Epuck/ChibiOS headers
#include "ch.h"
#include "msgbus/messagebus.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"

// Our headers


// Msg bus multi-threading tools
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Calibrated values of the system
static int tof_bias = 0;
static int proximity_bias = 0;
static int proximity_divider = 1;


/**************
 * Private  functions
 */

void getIRSensorsValues(proximity_msg_t* prox_values){
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    messagebus_topic_wait(prox_topic, prox_values, sizeof(*prox_values));
}

/**************
 * Public  functions (informations in the header)
 */

void mod_sensors_calibrateFrontSensor(int desiredValue){
    tof_bias = VL53L0X_get_dist_mm() - desiredValue;
}

int mod_sensors_getValueTOF(void){
    return (int) VL53L0X_get_dist_mm() - tof_bias;
    
}

void mod_sensors_initSensors(void){
    // TOF sensor
    VL53L0X_start();
    
    // Proximity sensors
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
}

void mod_sensors_stopSensors(void){
    // TOF sensor
    VL53L0X_stop();
}

void mod_sensors_calibrateIRSensors(int currentValue){
    
    static bool i=false;
    static int previousValue[2];
    
    proximity_msg_t prox_values;
    getIRSensorsValues(&prox_values);
    
    if(i == false){
        calibrate_ir();
        previousValue[0] = currentValue;
        previousValue[1] = prox_values.delta[0];
    }
    else if(i==true){
        proximity_divider = (prox_values.delta[0] - previousValue[1]) / (currentValue - previousValue[0]);
        proximity_bias = proximity_divider*previousValue[0]-previousValue[1];
    }
    i = !i;
    
}


void mod_sensors_getAllProximityValues(int* table){
    
    proximity_msg_t prox_values;
    getIRSensorsValues(&prox_values);
    
    int i=0;
    for(i=0; i<8;i++){
        table[i] = (prox_values.delta[i]+proximity_bias)/proximity_divider;
    }
}
