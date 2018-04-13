/*
 * File : mod_mapping.c
 * Project : e_puck_project
 * Description : Module that compute and store datas of the domain
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "mod_mapping.h"

#include "mod_sensors.h"
#include "mod_motors.h"
#include <ch.h>
#include <stdio.h>
#include <hal.h>
#include "math.h"
#include "mod_communication.h"

#define NUMBER_OF_MEASUREMENT_FOR_COMPLETE_ROTATION     20
#define WHEEL_SIZE                                      1
#define ROBOT_SIZE                                      1
#define COMPLETE_ANGLE                                  360
#define REDUCTION_CONST                                 100
#define CALIBRATED_CONST                                242

#define ARENA_WALL_DISTANCE                             66 // Between epuck and wall in the arena (in mm)

typedef struct {
    int x;
    int y;
    int theta;
}robotPosition_t;

typedef struct  {
    robotPosition_t position;
    int value;
}mesurement_t;

static robotPosition_t robotActualPosition;





/**************
 * Private functions
 */

/**
 * @brief Compute the position of the robot based on last position and the displacement (speed + time)
 *
 * @param[in] lastPosition     The position to be based on
 * @param[in] wheelSpeed       The wheel style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 *
 * @param[out] The new position of the robot
 */
robotPosition_t newAbsolutePositionWheelSpeedType(robotPosition_t* lastPosition,
                                                              const wheelSpeed_t* wheelSpeed, int time);

/**
 * @brief Compute the position of the robot based on last position and the displacement (speed + time)
 *
 * @param[in] lastPosition     The position to be based on
 * @param[in] robotSpeed       The robot style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 *
 * @param[out] The new position of the robot
 */
robotPosition_t newAbsolutePositionRobotSpeedType(robotPosition_t* lastPosition,
                                                              const robotSpeed_t* robotSpeed, int time);

/**
 * @brief Change motors states and compute the new position of the robot
 *
 * @param[in] wheelSpeed       The wheels speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
*/
void moveAndComputePositionWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime);

/**
 * @brief Change motors states and compute the new position of the robot
 *
 * @param[in] robotSpeed       The robot style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
 */
void moveAndComputePositionRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime);

/**
 * @brief Store the current value of the front distance sensor with the actual position of the robot
 *
 * @param[in] measurement       A pointer to the storing space
 */
void storeFrontDistanceSensorValue(mesurement_t* measurement);

/***************/

robotPosition_t newAbsolutePositionWheelSpeedType(robotPosition_t* lastPosition,
                                                              const wheelSpeed_t *wheelSpeed, int time){
    robotPosition_t newPosition;
    newPosition.x = lastPosition->x +   time*((wheelSpeed->right+wheelSpeed->left)/REDUCTION_CONST)
                                        *cos(lastPosition->theta)*CALIBRATED_CONST/REDUCTION_CONST/REDUCTION_CONST;
    newPosition.y = lastPosition->y +   time*((wheelSpeed->right+wheelSpeed->left)/REDUCTION_CONST)
                                        *sin(lastPosition->theta)*CALIBRATED_CONST/REDUCTION_CONST/REDUCTION_CONST;
    newPosition.theta = lastPosition->theta +   (wheelSpeed->right-wheelSpeed->left)*
                                                time/259/REDUCTION_CONST;
    return newPosition;
}


robotPosition_t newAbsolutePositionRobotSpeedType(robotPosition_t* lastPosition,
                                                              const robotSpeed_t* robotSpeed, int time){
    wheelSpeed_t wheelSpeed = mod_motors_convertRobotSpeedToWheelspeed(*robotSpeed);
    return newAbsolutePositionWheelSpeedType(lastPosition, &wheelSpeed, time);
}

void moveAndComputePositionWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime){
    mod_motors_changeStateWheelSpeedType(*wheelSpeed);
    systime_t time = chVTGetSystemTime();
    robotActualPosition = newAbsolutePositionWheelSpeedType(&robotActualPosition, wheelSpeed, time);
    chThdSleepUntilWindowed(time, time + MS2ST(deltaTime));
    mod_motors_stop();
}

void moveAndComputePositionRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime){
    wheelSpeed_t wheelSpeed = mod_motors_convertRobotSpeedToWheelspeed(*robotSpeed);
    moveAndComputePositionWheelSpeedType(&wheelSpeed, deltaTime);
}

void moveWheelSpeedType(const wheelSpeed_t* wheelSpeed, int deltaTime){
    mod_motors_changeStateWheelSpeedType(*wheelSpeed);
    systime_t time = chVTGetSystemTime();
    chThdSleepUntilWindowed(time, time + MS2ST(deltaTime));
    mod_motors_stop();
}

void moveRobotSpeedType(const robotSpeed_t* robotSpeed, int deltaTime){
    wheelSpeed_t wheelSpeed = mod_motors_convertRobotSpeedToWheelspeed(*robotSpeed);
    moveWheelSpeedType(&wheelSpeed, deltaTime);
}

void storeFrontDistanceSensorValue(mesurement_t* measurement){
    int value = mod_sensors_getValueTOF();
    *measurement = (mesurement_t) {robotActualPosition, value };
}

/**************
 * Public  functions (informations in the header)
 */


void mod_mapping_init(void){
    mod_sensors_initSensors();
    mod_motors_init();
    
    robotActualPosition.x = 0;
    robotActualPosition.y = 0;
    robotActualPosition.theta = 0;
    
}

void mod_mapping_calibrateTheSystem(void){
    mod_sensors_calibrateFrontSensor(ARENA_WALL_DISTANCE);
    mod_sensors_calibrateIRSensors(mod_sensors_getValueTOF());
    moveWheelSpeedType(&((wheelSpeed_t){200, 200}), 4000);
    chThdSleepMilliseconds(500);
    mod_sensors_calibrateIRSensors(mod_sensors_getValueTOF());
    moveWheelSpeedType(&((wheelSpeed_t){-200, -200}), 4000);
    
}


void mod_mapping_doInitialMapping(void){
    /*mesurement_t measurement[NUMBER_OF_MEASUREMENT_FOR_COMPLETE_ROTATION];
    storeFrontDistanceSensorValue(&(measurement[0]));
    //moveAndComputePositionWheelSpeedType(&((wheelSpeed_t){500, 500}), 9000);
    //storeFrontDistanceSensorValue(&(measurement[1]));
    //mod_motors_stop();
    chThdSleepMilliseconds(4000);
    moveAndComputePositionWheelSpeedType(&((wheelSpeed_t){-300, 300}), 10000);
    storeFrontDistanceSensorValue(&(measurement[2]));
    mod_motors_stop();
    char toSend[100];
    int i;
    for(i=0;i<3;i++){
        sprintf(toSend, "Computation %d : x %d | x %d | x %d | x %d", i, measurement[i].position.x, measurement[i].position.y,measurement[i].position.theta, measurement[i].value );
        mod_com_writeDatas(toSend, "TOUOE", 0);
    }*/
}




