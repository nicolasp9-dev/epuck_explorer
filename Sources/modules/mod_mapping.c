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
#include "mod_check.h"



#define ARENA_WALL_DISTANCE                             66// Between epuck and wall in the arena (in mm)
#define CALIBRATION_REF_TIME                            4000
#define ROTATION_ELMT_TIME                              50
#define TOLERATE_ERROR                                  4 // Error in mm

#define ROBOT_RADIUS    26.8f
#define TOF_RADIUS      33

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

void checkAngle(float *angle){
    if(*angle >= 2*M_PI){
        int nb = *angle/(2*M_PI);
        *angle -=  nb*2*M_PI;
    }
    while(*angle < 0){
        *angle += 2*M_PI;
    }
}

robotPosition_t newAbsolutePositionWheelSpeedType(robotPosition_t* lastPosition,
                                                              const wheelSpeed_t *wheelSpeed, int time){
    robotPosition_t newPosition;
    newPosition.x = lastPosition->x +   time*(wheelSpeed->right+wheelSpeed->left)*cos(lastPosition->theta)/(2*1000);
    newPosition.y = lastPosition->y +   time*(wheelSpeed->right+wheelSpeed->left)*sin(lastPosition->theta)/(2*1000);
    newPosition.theta = lastPosition->theta +   (wheelSpeed->right-wheelSpeed->left)*time/(ROBOT_RADIUS*2*1020);
    checkAngle(&newPosition.theta);
    return newPosition;
}

point_t measurementToPoint(measurement_t * measurement){
    point_t point;
    point.x = -(measurement->value+TOF_RADIUS)*sin(measurement->position.theta)+measurement->position.x;
    point.y = (measurement->value+TOF_RADIUS)*cos(measurement->position.theta)+measurement->position.y;
    return point;
}

void moveOrigin(point_t intersection, float thetaWall){
    robotActualPosition.x -= cos(thetaWall)*intersection.x - sin(thetaWall)*intersection.y;
    robotActualPosition.y -= sin(thetaWall)*intersection.x + cos(thetaWall)*intersection.y;
    robotActualPosition.theta -= thetaWall;
    if(robotActualPosition.theta >= 2*M_PI){
        int nb = robotActualPosition.theta/(2*M_PI);
        robotActualPosition.theta -=  nb*2*M_PI;
    }
    while(robotActualPosition.theta < 0){
        robotActualPosition.theta += 2*M_PI;
    }
}

void computeCoefDirecteur(point_t * point, int nbPoints, float * tot){
    float m, p, mtot = .0, ptot = .0;
    int i;
    for(i=0;i <(nbPoints-2);i++){
        m =  (float) (point[i].y-point[i+2].y) / (float) (point[i].x-point[i+2].x);
        p = (float) point[i].y - m * point[i].x;
        
        mtot += m;
        ptot += p;
    }
    tot[0] = mtot/i;
    tot[1] = ptot/i;
    
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


bool mod_mapping_computeWallLocation(measurement_t* measurement, int number){
    int i;
    int measureNumber = 0;
    int currentTable = 0;
    
    typedef enum {
        INCREASING =0,
        DECREASING,
        NOTHING
    } direction_t;
    
    typedef struct {
        measurement_t* measurement[NUMBER_OF_STEPS];
        direction_t direction;
        int total;
    } measurementsVariations_t;
    
    measurementsVariations_t * table = malloc(4*sizeof(measurementsVariations_t));
    assert(table);
    table[currentTable].direction = NOTHING;
    
    for(i=0; i < NUMBER_OF_STEPS; i++){
        if((measurement[i].value > 150) || (measurement[i].value < 1)) continue;
        
        if((table[currentTable].direction == NOTHING) && (measureNumber == 0)){
            table[currentTable].measurement[measureNumber] = &measurement[i];
        }
        
        else if(table[currentTable].direction == NOTHING) {
            if(measurement[i].value < table[currentTable].measurement[0]->value){
                table[currentTable].direction = DECREASING;
                table[currentTable].measurement[measureNumber] = &measurement[i];
            }
            else if(measurement[i].value == table[currentTable].measurement[0]->value){
                continue;
            }
            else{
                table[currentTable].direction = INCREASING;
                table[currentTable].measurement[measureNumber] = &measurement[i];
            }
        }
        else if(table[currentTable].direction == DECREASING) {
            if(measurement[i].value < table[currentTable].measurement[measureNumber-1]->value){
                table[currentTable].measurement[measureNumber] = &measurement[i];
            }
            else if(measurement[i].value == table[currentTable].measurement[measureNumber-1]->value){
                continue;
            }
            else{
                table[currentTable].total=measureNumber;
                currentTable++;
                measureNumber = 0;
                if(currentTable>3) break;
                table[currentTable].measurement[measureNumber] = &measurement[i];
                table[currentTable].direction = INCREASING;
            }
        }
        else if(table[currentTable].direction == INCREASING) {
            if(measurement[i].value > table[currentTable].measurement[measureNumber-1]->value){
                table[currentTable].measurement[measureNumber] = &measurement[i];
            }
            else if(measurement[i].value == table[currentTable].measurement[measureNumber-1]->value){
                continue;
            }
            else{
                table[currentTable].total=measureNumber;
                currentTable++;
                measureNumber = 0;
                if(currentTable>3) break;
                table[currentTable].measurement[measureNumber] = &measurement[i];
                table[currentTable].direction = DECREASING;
            }
        }
        
        measureNumber++;
    }
    table[currentTable].total=measureNumber;
    

    if(currentTable <3) return false;
    
    int j, k=0, l=0;
    point_t * wall1 = malloc(sizeof(point_t));
    point_t * wall2 = malloc(sizeof(point_t));
    for(i=0;i<4;i++){
        for(j=0;j<table[i].total;j++){
            if((i==0) || (i==1)){
                wall1 = realloc(wall1, (k+1)*sizeof(point_t));
                assert(wall1);
                
                wall1[k] = measurementToPoint(table[i].measurement[j]);
                
                k++;
            }
            else{
                wall2 = realloc(wall2, (l+1)*sizeof(point_t));
                assert(wall2);
                
                wall2[l] = measurementToPoint(table[i].measurement[j]);

                l++;
            }
        }
    }
    free(table);
    table = NULL;
    
    float coefsWall1[2], coefsWall2[2];
    
    computeCoefDirecteur(wall1, k, coefsWall1);
    computeCoefDirecteur(wall2, l, coefsWall2);
    
    free(wall1);
    wall1 = NULL;
    free(wall2);
    wall2 = NULL;
    
    
    point_t intersection;
    intersection.x = (coefsWall2[1] - coefsWall1[1]) /(coefsWall1[0] - coefsWall2[0]);
    intersection.y =  coefsWall2[0] * intersection.x + coefsWall2[1];
    
    float thetaWall = -atan(coefsWall2[0]);
    
    moveOrigin(intersection, thetaWall);
    
    return true;
    
}

robotDistance_t mod_mapping_getRobotDisplacement(const robotPosition_t * newAbsolutePosition){
    robotDistance_t displacement;
    
    int deltaX = newAbsolutePosition->x - robotActualPosition.x;
    int deltaY = newAbsolutePosition->y - robotActualPosition.y;
    float movementAngle = atan(deltaY/deltaX);
    
    displacement.rotation[0] = movementAngle - robotActualPosition.theta - M_PI/2;
    displacement.rotation[1] = newAbsolutePosition->theta - movementAngle+ M_PI/2;
    displacement.translation = sqrt(deltaX*deltaX + deltaY*deltaY);
    checkAngle(&displacement.rotation[0]);
    checkAngle(&displacement.rotation[1]);
    
    return displacement;
}

float mod_mapping_getAngleForTranslation(const float angle){
    float newAngle = angle - robotActualPosition.theta - M_PI/2;
    checkAngle(&newAngle);
    return newAngle;
}

robotPosition_t mod_mapping_getActualPosition(void){
    return robotActualPosition;
}




