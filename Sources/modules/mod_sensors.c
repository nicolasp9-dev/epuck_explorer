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
#include "mod_communication.h"

#define OBJECT_DECTECTION_FREQUENCY         400
#define OBSTACLE_DISTANCE                   20


// Msg bus multi-threading tools
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Calibrated values of the system
static int tof_bias = 3;
static int proximity_bias = 0;
static int proximity_multiplier = 1;

// Threads objects
static thread_t * obstacleThread;

// Semaphores
binary_semaphore_t isObstacle_sem;

/**************
 * Private  functions
 */

void getIRSensorsValues(proximity_msg_t* prox_values){
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    messagebus_topic_wait(prox_topic, prox_values, sizeof(*prox_values));
}

static THD_WORKING_AREA(objectDetectionSensor_wa, 1024);
static THD_FUNCTION(objectDetectionSensor, arg){
    (void) arg;
    int tab[8];
    mod_sensors_getAllProximityValues(tab);
    int i;
    while(1){
        for(i=0;i < 8;i++){
            if(tab[i] < OBSTACLE_DISTANCE){
                chSysLockFromISR();
                chSemSignalI(&isObstacle_sem);
                chSysUnlockFromISR();
            }
        }
        chThdSleepMilliseconds(OBJECT_DECTECTION_FREQUENCY);
    }
}



/**************
 * Public  functions (informations in the header)
 */

void mod_sensors_initSensors(void){
    
    chBSemObjectInit(&isObstacle_sem, false);
    
    // TOF sensor
    VL53L0X_start();
    
    // Proximity sensors
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
    chThdSleepMilliseconds(500);
}

void mod_sensors_initCalibration(void){
    tof_bias = 0;
    proximity_bias = 0;
    proximity_multiplier = 1;
}

/*
 * TOF Functions
 */

void mod_sensors_calibrateFrontSensor(int desiredValue){
    tof_bias = (int) VL53L0X_get_dist_mm() - desiredValue;
}

int mod_sensors_getValueTOF(void){
    return (int) VL53L0X_get_dist_mm() - tof_bias;
    
}

void mod_sensors_stopTOF(void){
    VL53L0X_stop();
}

/*
 * IR Sensors Functions
 */

void mod_sensors_initObjectDetection(void){
    obstacleThread = chThdCreateStatic(objectDetectionSensor_wa, sizeof(objectDetectionSensor_wa), NORMALPRIO+2, objectDetectionSensor, NULL);
}

void mod_sensors_getAllProximityValues(int* table){
    
    proximity_msg_t prox_values;
    getIRSensorsValues(&prox_values);
    
    int i=0;
    for(i=0; i<8;i++){
        table[i] = (prox_values.delta[i]+proximity_bias)*proximity_multiplier;
    }
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
        proximity_multiplier = (currentValue - previousValue[0])/ (prox_values.delta[0] - previousValue[1]);
        proximity_bias = previousValue[0]-previousValue[1]/proximity_multiplier;
        char toSend[100];
        sprintf(toSend, "tof_bias %d \n", tof_bias);
        mod_com_writeDatas(toSend, "0", 0);
        sprintf(toSend, "proximity_bias %d \n", proximity_bias);
        mod_com_writeDatas(toSend, "0", 0);
        sprintf(toSend, "proximity_divider %d \n", proximity_multiplier);
        mod_com_writeDatas(toSend, "0", 0);
    }
    i = !i;
    
}
