/*
 * File : mod_mapping.c
 * Project : e_puck_project
 * Description : Module that compute and store datas of the domain
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "mod_mapping.h"

#include <ch.h>
#include <stdio.h>
#include <hal.h>
#include "math.h"
#include "mod_communication.h"
#include "mod_basicIO.h"



#define ARENA_WALL_DISTANCE                             66// Between epuck and wall in the arena (in mm)
#define CALIBRATION_REF_TIME                            4000
#define ROTATION_ELMT_TIME                              50
#define TOLERATE_ERROR                                  4 // Error in mm

#define ROBOT_RADIUS    27
#define TOF_RADIUS      37

static robotPosition_t robotActualPosition;

/********************
 *  Private functions
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
robotPosition_t PositionWheelSpeedType(robotPosition_t* lastPosition,
                                                  const wheelSpeed_t* wheelSpeed, int time);


/***************/

robotPosition_t newAbsolutePositionWheelSpeedType(robotPosition_t* lastPosition,
                                                              const wheelSpeed_t *wheelSpeed, int time){
    robotPosition_t newPosition;
    newPosition.x = lastPosition->x +   time*(wheelSpeed->right+wheelSpeed->left)*cos(lastPosition->theta)/(2*1000);
    newPosition.y = lastPosition->y +   time*(wheelSpeed->right+wheelSpeed->left)*sin(lastPosition->theta)/(2*1000);
    newPosition.theta = lastPosition->theta +   (wheelSpeed->right-wheelSpeed->left)*time/ROBOT_RADIUS;
    if(newPosition.theta > 2*M_PI){
        newPosition.theta -=  M_PI;
    }
    return newPosition;
}

point_t measurementToPoint(measurement_t * measurement){
    point_t point;
    point.x = -(measurement->value+TOF_RADIUS)*sin(robotActualPosition.theta)+robotActualPosition.x;
    point.y = -(measurement->value+TOF_RADIUS)*cos(robotActualPosition.theta)+robotActualPosition.y;
    return point;
}


/**************
 * Public  functions (informations in the header)
 */


void mod_mapping_init(void){

    mod_mapping_resetCoordinates();
}

void mod_mapping_resetCoordinates(void){
    robotActualPosition = (robotPosition_t) {0,0,0};
}

void mod_mapping_adaptCoordinatesToOrigin(int x, int y, int theta){
    robotActualPosition = (robotPosition_t) {robotActualPosition.x,robotActualPosition.x,robotActualPosition.theta};
}

void mod_mapping_updatePositionWheelSpeedType(const wheelSpeed_t *wheelSpeed, int time){
    robotActualPosition = newAbsolutePositionWheelSpeedType(&robotActualPosition, wheelSpeed, time);
}


void mod_mapping_computeWallLocation(measurement_t* measurement, int number){
    int i;
    int measureNumber = 0;
    int currentTable = 0;
    
    typedef enum {
        INCREASING =0,
        DECREASING,
        NOTHING
    } direction_t;
    
    typedef struct {
        measurement_t measurement[NUMBER_OF_STEPS];
        direction_t direction;
        int total;
    } measurementsVariations_t;
    
    measurementsVariations_t table[4];
    
    for(i=0; i < NUMBER_OF_STEPS; i++){
        if(measurement[i].value > 120) continue;
        
        if((table[currentTable].direction == NOTHING) && (measureNumber == 0)){
            table[currentTable].measurement[measureNumber] = measurement[i];
        }
        
        else if(table[currentTable].direction == NOTHING) {
            if(measurement[i].value < table[currentTable].measurement[0].value){
                table[currentTable].direction = DECREASING;
                table[currentTable].measurement[measureNumber] = measurement[i];
            }
            else{
                table[currentTable].direction = INCREASING;
                table[currentTable].measurement[measureNumber] = measurement[i];
            }
        }
        else if(table[currentTable].direction == DECREASING) {
            if(measurement[i].value < table[currentTable].measurement[measureNumber-1].value){
                table[currentTable].measurement[measureNumber] = measurement[i];
            }
            else{
                table[currentTable].total=measureNumber;
                currentTable++;
                measureNumber = 0;
                if(currentTable>3) break;
                table[currentTable].measurement[measureNumber] = measurement[i];
                table[currentTable].direction = INCREASING;
            }
        }
        else if(table[currentTable].direction == INCREASING) {
            if(measurement[i].value > table[currentTable].measurement[measureNumber-1].value){
                table[currentTable].measurement[measureNumber] = measurement[i];
            }
            else{
                table[currentTable].total=measureNumber;
                currentTable++;
                measureNumber = 0;
                if(currentTable>3) break;
                table[currentTable].measurement[measureNumber] = measurement[i];
                table[currentTable].direction = DECREASING;
            }
        }
        
        measureNumber++;
    }
    int j, k=0, l=0;
    point_t wall1[NUMBER_OF_STEPS];
    point_t wall2[NUMBER_OF_STEPS];
    for(i=0;i<4;i++){
        for(j=0;j<table[i].total;j++){
            if((i==0) || (i==1)){
                wall1[k] = measurementToPoint(&(table[i].measurement[j]));
                k++;
            }
            else{
                wall2[l] = measurementToPoint(&(table[i].measurement[j]));
                l++;
            }
        }
    }
    
}

robotDistance_t mod_mapping_getRobotDisplacement(const robotPosition_t * newAbsolutePosition){
    robotDistance_t displacement;
    
    int deltaX = newAbsolutePosition->x - robotActualPosition.x;
    int deltaY = newAbsolutePosition->y - robotActualPosition.y;
    float movementAngle = atan(deltaY/deltaX);
    
    displacement.rotation[0] = movementAngle - robotActualPosition.theta;
    displacement.rotation[1] = newAbsolutePosition->theta - movementAngle;
    displacement.translation = sqrt(deltaX*deltaX + deltaY*deltaY);
    
    return displacement;
}

robotPosition_t mod_mapping_getActualPosition(void){
    return robotActualPosition;
}




