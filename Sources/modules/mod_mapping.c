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
#define TOLERANCE_WALL                                  40
#define TOLERANCE_OBJECT                                25
#define PICTURE_DISTANCE                                80

#define ROBOT_RADIUS    27
#define TOF_RADIUS      33
#define NUMBER_OF_WALLS 4

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

typedef struct {
    int x0;
    int x2;
    int y1;
    int y3;
} wall_t;

wall_t wall;

point_t *objectList;
int objectListSize=0;

/***************/

void checkAngle(double *angle){
    while(*angle >= 2*M_PI) *angle -= 2*M_PI;
    while(*angle < 0) *angle += 2*M_PI;
}

robotPosition_t newAbsolutePositionWheelSpeedType(robotPosition_t* lastPosition,
                                                              const wheelSpeed_t *wheelSpeed, int time){
    robotPosition_t newPosition;
    newPosition.x = lastPosition->x +   time*(wheelSpeed->right+wheelSpeed->left)*cos(lastPosition->theta)/(2*1000);
    newPosition.y = lastPosition->y +   time*(wheelSpeed->right+wheelSpeed->left)*sin(lastPosition->theta)/(2*1000);
    newPosition.theta = lastPosition->theta +   (wheelSpeed->right-wheelSpeed->left)*time/(ROBOT_RADIUS*2*1010);
    checkAngle(&newPosition.theta);
    
    char toSend[100];
    sprintf(toSend, "Robot new position : %d, %d, %f", newPosition.x, newPosition.y, newPosition.theta);
    mod_com_writeMessage(toSend,3);
    
    return newPosition;
}

point_t measurementToPoint(measurement_t * measurement){
    point_t point;
    point.x = -(measurement->value+TOF_RADIUS)*sin(measurement->position.theta)+measurement->position.x;
    point.y = (measurement->value+TOF_RADIUS)*cos(measurement->position.theta)+measurement->position.y;
    
    char toSend[100];
    sprintf(toSend, "Point location : %d, %d", point.x, point.y);
    mod_com_writeMessage(toSend,3);
    
    return point;
}

void moveOrigin(point_t intersection, double thetaWall){
    robotActualPosition.x -= cos(thetaWall)*intersection.x - sin(thetaWall)*intersection.y;
    robotActualPosition.y -= sin(thetaWall)*intersection.x + cos(thetaWall)*intersection.y;
    robotActualPosition.theta -= thetaWall + M_PI/2;
    if(robotActualPosition.theta >= 2*M_PI){
        int nb = robotActualPosition.theta/(2*M_PI);
        robotActualPosition.theta -=  nb*2*M_PI;
    }
    while(robotActualPosition.theta < 0){
        robotActualPosition.theta += 2*M_PI;
    }
    
    char toSend[100];
    sprintf(toSend, "RMoved origin : %d, %d, %f", robotActualPosition.x, robotActualPosition.y, robotActualPosition.theta);
    mod_com_writeMessage(toSend,3);
}

void computeCoefDirecteur(point_t * point, int nbPoints, double * tot){
    double m, p, mtot = .0, ptot = .0;
    int i;
    for(i=0;i <(nbPoints-3);i++){
        m =  (double) (point[i].y-point[i+3].y) / (double) (point[i].x-point[i+3].x);
        p = (double) point[i].y - m * point[i].x;
        
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
    
    objectList = malloc(sizeof(point_t));
    assert(objectList);
}

void mod_mapping_resetCoordinates(void){
    robotActualPosition = (robotPosition_t) {0,0,0};
}

void mod_mapping_adaptCoordinatesToOrigin(int x, int y, int theta){
    robotActualPosition = (robotPosition_t) {x,x,theta};
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
    
    double coefsWall1[2], coefsWall2[2];
    
    computeCoefDirecteur(wall1, k, coefsWall1);
    computeCoefDirecteur(wall2, l, coefsWall2);
    
    free(wall1);
    wall1 = NULL;
    free(wall2);
    wall2 = NULL;
    
    
    point_t intersection;
    intersection.x = (coefsWall2[1] - coefsWall1[1]) /(coefsWall1[0] - coefsWall2[0]);
    intersection.y =  coefsWall2[0] * intersection.x + coefsWall2[1];
    
    double thetaWall = (-atan(coefsWall2[0]) + atan(1/coefsWall1[0]))/2;

    moveOrigin(intersection, thetaWall);
    
    wall.x0 = 0;
    wall.y1 = 0;
    
    return true;
    
}

void mod_map_saveWall(measurement_t value, int wallNum){
    if(wallNum == 2){
        wall.x2 = measurementToPoint(&value).x;
    }
    if(wallNum == 3){
        wall.y3 = measurementToPoint(&value).y;
    }

    
}

robotDistance_t mod_mapping_getRobotDisplacement(const robotPosition_t * newAbsolutePosition){
    robotDistance_t displacement;
    
    int deltaX = newAbsolutePosition->x - robotActualPosition.x;
    int deltaY = newAbsolutePosition->y - robotActualPosition.y;
    double movementAngle = atan(deltaY/deltaX);
    
    displacement.rotation[0] = movementAngle - robotActualPosition.theta - M_PI/2;
    displacement.rotation[1] = newAbsolutePosition->theta - movementAngle+ M_PI/2;
    displacement.translation = sqrt(deltaX*deltaX + deltaY*deltaY);
    checkAngle(&displacement.rotation[0]);
    checkAngle(&displacement.rotation[1]);
    
    char toSend[100];
    sprintf(toSend, "Displacement to do : %f, %d, %f", displacement.rotation[0], displacement.translation, displacement.rotation[1]);
    mod_com_writeMessage(toSend,3);
    
    return displacement;
}

double mod_mapping_getAngleForTranslation(const double angle){
    double newAngle = angle - robotActualPosition.theta - M_PI/2;
    
    while(newAngle > M_PI) newAngle -= 2*M_PI;
    while(newAngle < -M_PI) newAngle += 2*M_PI;
    
    char toSend[50];
    sprintf(toSend, "Angle to take : %f", newAngle);
    mod_com_writeMessage(toSend,3);
    
    return newAngle;
}

robotPosition_t mod_mapping_getActualPosition(void){
    return robotActualPosition;
}

int computeObjectDistance(point_t point1, point_t point2){
    return sqrt((point1.x-point2.x)*(point1.x-point2.x) + (point1.y-point2.y)*(point1.y-point2.y));
}

void computeObjectRelativeDirection(point_t point1, point_t point2){
    
}

bool isNear(point_t point){
    if(computeObjectDistance(point, (point_t) {robotActualPosition.x, robotActualPosition.y}) < TOF_RADIUS + TOLERANCE_WALL*2){
        return true;
    }
    return false;
}

bool checkIfPointInCircle(point_t point, point_t circle){
    if(((point.x-circle.x)*(point.x-circle.x) + (point.y-circle.y)*(point.y-circle.y)) < TOLERANCE_OBJECT) return true;
    else return false;
    
}

bool checkIfObjectExists(point_t object){
    for(int i = 0; i < objectListSize; i++)
        if(checkIfPointInCircle(object, objectList[i]))
            return true;
    return false;
}


void mod_mapping_checkEnvironment(measurement_t * measurement, int numberOfMeasurements, actualEnvironement_t * environmentObstacles){
    point_t * point = malloc(numberOfMeasurements*sizeof(point_t));
    assert(point);
    environmentObstacles->numberOfknownObjects = 0;
    environmentObstacles->numberOfnewObjects = 0;
    
    for(int i=0; i< NUMBER_OF_WALLS; i++){
        environmentObstacles->nearWall[i] = false;
    }
    
    for(int i=0; i< numberOfMeasurements; i++){
        
        point[i] = measurementToPoint(&measurement[i]);
        
        if(!isNear(point[i])) continue;
            
             if(point[i].x < wall.x0 + TOLERANCE_WALL) environmentObstacles->nearWall[0] = true;
        else if(point[i].x > wall.x2 - TOLERANCE_WALL) environmentObstacles->nearWall[1] = true;
        else if(point[i].y < wall.y1 + TOLERANCE_WALL) environmentObstacles->nearWall[2] = true;
        else if(point[i].y > wall.y3 - TOLERANCE_WALL) environmentObstacles->nearWall[3] = true;
        
        else{
            if(checkIfObjectExists(point[i])){
                environmentObstacles->knownObjectsLocation[environmentObstacles->numberOfknownObjects] = point[i];
                environmentObstacles->numberOfknownObjects++;
            }
            else{
                environmentObstacles->newObjectsLocation[environmentObstacles->numberOfnewObjects] = point[i];
                environmentObstacles->numberOfnewObjects++;
            }
            if((environmentObstacles->numberOfnewObjects == 3) || (environmentObstacles->numberOfknownObjects == 3)){
                break;
            }
        }
           
    }
    
    free(point);
    point = NULL;
    
}

point_t getEmplacementForPicture(point_t point){

    
}

void getPathTo(){
    
}

