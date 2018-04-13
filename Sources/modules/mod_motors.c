/*
 * File : mod_motors.c
 * Project : e_puck_project
 * Description : Module that manages actuators
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "mod_motors.h"
#include "motors.h"
#include "mod_mapping.h"
#include <arm_math.h>

#define MAX_SPEED       1100
#define MAX_DIRECTION   1000


wheelSpeed_t mod_motors_convertRobotSpeedToWheelspeed(robotSpeed_t robotSpeedTemp){
    
    if(robotSpeedTemp.direction > MAX_DIRECTION)
        robotSpeedTemp.direction = MAX_DIRECTION;
    else if(robotSpeedTemp.direction < -MAX_DIRECTION)
        robotSpeedTemp.direction = -MAX_DIRECTION;
    
    if(robotSpeedTemp.mainSpeed > MAX_SPEED)
        robotSpeedTemp.mainSpeed = MAX_SPEED;
    else if(robotSpeedTemp.mainSpeed < -MAX_SPEED)
        robotSpeedTemp.mainSpeed = -MAX_SPEED;
    
    wheelSpeed_t wheelSpeed = { -robotSpeedTemp.mainSpeed *
                                (robotSpeedTemp.direction-MAX_DIRECTION)/MAX_DIRECTION,
                                robotSpeedTemp.mainSpeed *
                                (robotSpeedTemp.direction+MAX_DIRECTION)/MAX_DIRECTION};
    return wheelSpeed;
}


void mod_motors_init(void){
    motors_init();
}

void mod_motors_changeStateWheelSpeedType(wheelSpeed_t wheelSpeed){
    left_motor_set_speed(wheelSpeed.left);
    right_motor_set_speed(wheelSpeed.right);
}

void mod_motors_changeStateRobotSpeedType(robotSpeed_t robotSpeed){
    mod_motors_changeStateWheelSpeedType(mod_motors_convertRobotSpeedToWheelspeed(robotSpeed));
}


void mod_motors_calibrateDistance(void){
    
    
}


void mod_motors_stop(void){
    mod_motors_changeStateWheelSpeedType((wheelSpeed_t){0,0});
}
