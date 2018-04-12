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
#include <hal.h>
#include "math.h"

#define NUMBER_OF_MEASUREMENT_FOR_COMPLETE_ROTATION     50
#define WHEEL_SIZE                                      1
#define ROBOT_SIZE                                      1
#define COMPLETE_ANGLE                                  360

typedef struct  {
    int left;
    int right;
}wheelSpeed_t;

typedef struct  {
    int angle;
    int mainSpeed;
}robotSpeed_t;

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


/*
 * Prototypes of local function
 */
wheelSpeed_t mod_mapping_convertWheelspeed(robotSpeed_t robotSpeed);

robotPosition_t mod_mapping_newAbsolutePosition(robotPosition_t lastPosition, robotSpeed_t robotSpeed, int time);

void mod_mapping_moveAndComputePosition(robotSpeed_t robotSpeed, int deltaTime);

void mod_mapping_storeWallPosition(void);





void mod_mapping_init(void){
    robotActualPosition.x = 0;
    robotActualPosition.y = 0;
    robotActualPosition.theta = 0;
    
}

void mod_mapping_doInitialMapping(void){
    do{
        mod_mapping_storeWallPosition();
        mod_mapping_moveAndComputePosition((robotSpeed_t){1000, 400}, 1000);
    }while(robotActualPosition.theta < COMPLETE_ANGLE );
}

void mod_mapping_storeWallPosition(void){
    static mesurement_t measurement[NUMBER_OF_MEASUREMENT_FOR_COMPLETE_ROTATION];
    static int i = 0;
    measurement[i] = (mesurement_t) {robotActualPosition, mod_sensor_getValueLightSensor() };
    i++;
}

void mod_mapping_moveAndComputePosition(robotSpeed_t robotSpeed, int deltaTime){
    mod_motors_state(robotSpeed.angle, robotSpeed.mainSpeed);
    systime_t time = chVTGetSystemTime();
    robotActualPosition = mod_mapping_newAbsolutePosition(robotActualPosition, robotSpeed, time);
    chThdSleepUntilWindowed(time, time + MS2ST(deltaTime));
}

robotPosition_t mod_mapping_newAbsolutePosition(robotPosition_t lastPosition, robotSpeed_t robotSpeed, int time){
    wheelSpeed_t wheelSpeed = mod_mapping_convertWheelspeed(robotSpeed);
    robotPosition_t newPosition;
    newPosition.x =lastPosition.x + WHEEL_SIZE*(cos(lastPosition.theta)*wheelSpeed.right+
                                                cos(lastPosition.theta)*wheelSpeed.left)*time;
    newPosition.y = lastPosition.y + WHEEL_SIZE*(sin(lastPosition.theta)*wheelSpeed.right+
                                                 sin(lastPosition.theta)*wheelSpeed.left)*time;
    newPosition.theta = lastPosition.theta + WHEEL_SIZE*(wheelSpeed.right/ROBOT_SIZE-
                                                         wheelSpeed.left/ROBOT_SIZE)*time;
    return newPosition;
}

wheelSpeed_t mod_mapping_convertWheelspeed(robotSpeed_t robotSpeed){
    wheelSpeed_t wheelSpeed;
    wheelSpeed.left = -robotSpeed.mainSpeed * (robotSpeed.angle-1000)/1000;
    wheelSpeed.right = robotSpeed.mainSpeed * (robotSpeed.angle+1000)/1000;
    return wheelSpeed;
}
