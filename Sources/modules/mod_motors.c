/*
 * File : mod_motors.c
 * Project : e_puck_project
 * Description : Module that manages actuators
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

// Warning ! All speed are in mm/s

#include "mod_motors.h"
#include "motors.h"
#include "mod_mapping.h"
#include <arm_math.h>

#include <stdio.h>
#include "mod_communication.h"

static int angle_const = 1;
static int straight_const = 31;

void mod_motors_init(void){
    motors_init();
}

wheelSpeed_t mod_motors_convertRobotSpeedToWheelspeed(robotSpeed_t robotSpeedTemp){
    return (wheelSpeed_t) {robotSpeedTemp.mainSpeed + robotSpeedTemp.angle*ROBOT_RADIUS/2,
                           robotSpeedTemp.mainSpeed - robotSpeedTemp.angle*ROBOT_RADIUS/2 };
}

robotSpeed_t mod_motors_convertWheelSpeedToMotorspeed(wheelSpeed_t wheelSpeedTemp){
    return (robotSpeed_t) { (wheelSpeedTemp.left + wheelSpeedTemp.right) /2,
                            (wheelSpeedTemp.right - wheelSpeedTemp.left) /ROBOT_RADIUS};
    
}


void mod_motors_initCalibration(void){
    angle_const = 1;
    straight_const = 1;
}
void mod_motor_angleCalibration(const wheelSpeed_t displacement, const int movementTime){
    angle_const = (displacement.left - displacement.right)*movementTime/(2*PI1000*ROBOT_RADIUS);
}

void mod_motor_distanceCalibration(int distanceWithWall, const wheelSpeed_t displacement, const int movementTime){
    static bool i=false;
    static int previousValue;

    if(i == false){
        previousValue = distanceWithWall;
        char toSend[100];
        sprintf(toSend, "prev1 : %d", previousValue);
        mod_com_writeDatas(toSend, "HAHA", 0);
    }
    else if(i==true){
        int realSpeed = (previousValue - distanceWithWall)*MS_TO_S/movementTime;
        int sentSpeed = (displacement.right + displacement.left)/2;
        straight_const = sentSpeed / realSpeed;
        char toSend[100];
        sprintf(toSend, "Straight const : %d preval : %d distWwall : %d displ : %d", straight_const, previousValue, distanceWithWall, displacement.right);
        mod_com_writeDatas(toSend, "HAHA", 0);
    }
    i = !i;
}

    

void mod_motors_changeStateWheelSpeedType(wheelSpeed_t wheelSpeed){
    left_motor_set_speed(wheelSpeed.left*straight_const);
    right_motor_set_speed(wheelSpeed.right*straight_const);
}

void mod_motors_changeStateRobotSpeedType(robotSpeed_t robotSpeed){
    mod_motors_changeStateWheelSpeedType(mod_motors_convertRobotSpeedToWheelspeed(robotSpeed));
}


void mod_motors_stop(void){
    mod_motors_changeStateWheelSpeedType((wheelSpeed_t){0,0});
}
