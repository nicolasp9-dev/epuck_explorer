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
#include <ch.h>

typedef struct{
    int BiasLeftSpeed;
    int BiasRightSpeed;
} speedBias_t;

typedef enum {
    IGNORE_RIGHT,
    IGNORE_LEFT,
    IGNORE_NOTHING,
} ignore_t;

extern binary_semaphore_t isObstacle_sem;

/**
 * @brief Initialize proximity and TOF sensors, to be ready for use
 */
void mod_sensors_initSensors(void);

/**
 * @brief Initialize static values to have a good calibration
 */
void mod_sensors_initCalibration(void);

/**
 * @brief Calibrate the tof device applying a bias of the difference between actual data and desired one.
 *
 * @param[in] desiredValue     The current measured value of the distance
 */
void mod_sensors_calibrateFrontSensor(int desiredValue);

/**
 * @brief Get the value of the TOF sensor (in mm)
 *
 * @param[out] The value of the sensor, taking in account a possible bias
 */
int mod_sensors_getValueTOF(void);

/**
 * @brief Stop TOF sensors
 */
void mod_sensors_stopTOF(void);

/**
 * @brief TO DO
 */
void mod_sensors_initObjectDetection(void);

/**
 * @brief Compute all values of IR sensors with calibrated datas (in mm)
 *
 * @param[in] table A table where to save datas
 */
void mod_sensors_getAllProximityValues(int* table);

/**
 * @brief 2 steps calibration of the IR sensor, the robot need to move straight ahead before the second step (to have two different measures)
 * @note This function must be called to time, one before the displacement, and the second after it
 *
 * @param[in] currentValue The distance between the robot and the wall in front of the robot
 */
void mod_sensors_calibrateIRSensors(int currentValue);






#endif
