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
#define TOLERANCE_OBJECT                                60
#define TOLERANCE_OBJECT_BIS                            20
#define PICTURE_DISTANCE                                100

#define ROBOT_RADIUS    27
#define TOF_RADIUS      33
#define NUMBER_OF_WALLS 4

static robotPosition_t robotActualPosition;


// Points of found objects
typedef struct {
    int x0;
    int x2;
    int y1;
    int y3;
} wall_t;

wall_t wall;

actualEnvironement_t environment;

// Points of found objects
point_t *objectList;
int objectListSize=0;



/********************
 *  Private functions
 */


/**
 * @brief Check if the angle is between 0 -> 2PI, if not, changes it
 *
 * @param[in] angle        A pointer to the angle value
 */
void checkAngle(float *angle);

/**
 * @brief Compute the new position of the robot based on a position and a displacement
 *
 * @param[in] lastPosition        The last position
 * @param[in] wheelSpeed          The speed of the displacement
 * @param[in] time                The time of the movement
 */
robotPosition_t newAbsolutePositionWheelSpeedType(robotPosition_t* lastPosition,
                                                  const wheelSpeed_t *wheelSpeed, int time);


/**
 * @brief Returns the point corresponding to the object in the actual coordinates system
 *
 * @param[in] measurement          The measurement to analyse
 */
point_t measurementToPoint(measurement_t * measurement);

/**
 * @brief Changes the origin location based on robot measures
 *
 * @param[in] intersection       The intersection of wall point
 * @param[in] thetaWall          The wall angle
 */
void moveOrigin(point_t intersection, float thetaWall);



/**
 * @brief Computes straight line coefs based on points
 *
 * @param[in] point       The list of points
 * @param[in] nbPoints    The number of points
 * @param[in] tot         The table where to store coefs (y = a*x + b, tot[0] = a, tot[1] = b)
 */
void computeCoefDirecteur(point_t * point, int nbPoints, float * tot);


/**
 * @brief Computes the distance between two points
 *
 * @param[in] point1      The first point
 * @param[in] point2      The second point
 *
 * @param[out] The distance
 */
int computeObjectDistance(point_t point1, point_t point2);


/**
 * @brief Check if a point is in the circle around another point (defined by TOLERANCE_OBJECT)
 *
 * @param[in] point1      The point
 * @param[in] circle      The object
 *
 * @param[out] True if it's in
 */
bool checkIfPointObjectInCircle(point_t point, point_t circle);


/**
 * @brief Check if the object is in the object list
 *
 * @param[in] object      The object to check
 *
 * @param[out] True if it's in
 */
bool checkIfObjectExists(point_t object);


/**
 * @brief Says if a point have an interest and needs to be analyse
 *
 * @param[in] point      The point to analyse
 *
 * @param[out] True if it's near
 */
bool isNear(point_t point);

/***************/


void checkAngle(float *angle){
    while(*angle >= 2*M_PI) *angle -= 2*M_PI;
    while(*angle < 0) *angle += 2*M_PI;
}


robotPosition_t newAbsolutePositionWheelSpeedType(robotPosition_t* lastPosition,
                                                  const wheelSpeed_t *wheelSpeed, int time){
    robotPosition_t newPosition;
    newPosition.x = lastPosition->x +   time*(wheelSpeed->right+wheelSpeed->left)*cos(lastPosition->theta+M_PI/2)/(2*1000);
    newPosition.y = lastPosition->y +   time*(wheelSpeed->right+wheelSpeed->left)*sin(lastPosition->theta+M_PI/2)/(2*1000);
    newPosition.theta = lastPosition->theta +   (wheelSpeed->right-wheelSpeed->left)*time/(ROBOT_RADIUS*2*1010);
    
    
    checkAngle(&newPosition.theta);
    
    return newPosition;
}

point_t measurementToPoint(measurement_t * measurement){
    point_t point;
    point.x = -(measurement->value+TOF_RADIUS)*sin(measurement->position.theta)+measurement->position.x;
    point.y = (measurement->value+TOF_RADIUS)*cos(measurement->position.theta)+measurement->position.y;
    char toSend[50];
    sprintf(toSend, "New point computed : %d, %d", point.x, point.y);
    mod_com_writeMessage(toSend, 3);
    return point;
}


void moveOrigin(point_t intersection, float thetaWall){
    robotActualPosition.x -= cos(thetaWall)*intersection.x - sin(thetaWall)*intersection.y;
    robotActualPosition.y -= sin(thetaWall)*intersection.x + cos(thetaWall)*intersection.y;
    robotActualPosition.theta += thetaWall ;
    checkAngle(&robotActualPosition.theta);
    char toSend[50];
    sprintf(toSend, "New origin: %d, %d, %f", robotActualPosition.x, robotActualPosition.y, robotActualPosition.theta);
    mod_com_writeMessage(toSend, 3);
}


void computeCoefDirecteur(point_t * point, int nbPoints, float * tot){
    float m, p, mtot = .0, ptot = .0;
    int i;
    for(i=0;i <(nbPoints-2);i++){
        m =  (float) (point[i].y-point[i+2].y) / (float) (point[i].x-point[i+2].x);
        p = (float) point[i].y - m * point[i].x;
        
        char toSend[50];
        sprintf(toSend, "m: %d / %d nb %d", (point[i].y-point[i+2].y), (point[i].x-point[i+2].x), nbPoints);
        mod_com_writeMessage(toSend, 3);
        
        mtot += m;
        ptot += p;
    }
    tot[0] = mtot/i;
    tot[1] = ptot/i;
}

int computeObjectDistance(point_t point1, point_t point2){
    return sqrt((point1.x-point2.x)*(point1.x-point2.x) + (point1.y-point2.y)*(point1.y-point2.y));
}

bool checkIfPointObjectInCircle(point_t point, point_t circle){
    if(((point.x-circle.x)*(point.x-circle.x) + (point.y-circle.y)*(point.y-circle.y)) < TOLERANCE_OBJECT) return true;
    else return false;
    
}

bool checkIfObjectExists(point_t object){
    for(int i = 0; i < objectListSize; i++)
        if(checkIfPointObjectInCircle(object, objectList[i]))
            return true;
    return false;
}

bool isNear(point_t point){
    if(computeObjectDistance(point, (point_t) {robotActualPosition.x, robotActualPosition.y}) < TOF_RADIUS + 150){
        return true;
    }
    return false;
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


void mod_mapping_updatePositionWheelSpeedType(const wheelSpeed_t *wheelSpeed, int time){
    robotActualPosition = newAbsolutePositionWheelSpeedType(&robotActualPosition, wheelSpeed, time);
    char toSend[50];
    sprintf(toSend, "New position: %d, %d, %f", robotActualPosition.x, robotActualPosition.y, robotActualPosition.theta);
    mod_com_writeMessage(toSend, 3);
}


bool mod_mapping_computeWallLocation(measurement_t* measurement){
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
    
    measurementsVariations_t * table = malloc(8*sizeof(measurementsVariations_t));
    assert(table);
    table[currentTable].direction = NOTHING;
    for(i=0; i < NUMBER_OF_STEPS; i++){
        if(measurement[i].value < 50) continue;

        
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
                if(currentTable>7) break;
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
                if(currentTable>7) break;
                table[currentTable].measurement[measureNumber] = &measurement[i];
                table[currentTable].direction = DECREASING;
            }
        }
        measureNumber++;
    }
    table[currentTable].total=measureNumber;
    

    if(currentTable <3) return false;
    
    int j;
    float coefsWall[NUMBER_OF_WALLS][2];
    
    {
        point_t * wall[NUMBER_OF_WALLS];
        int step[NUMBER_OF_WALLS] ={0,0,0,0};
        
        for(int i =0; i <NUMBER_OF_WALLS;i++) wall[i] = malloc(sizeof(point_t));
        int id;
        for(i=0;i<8;i++){
            id = i/2;
            for(j=0;j<table[i].total;j++){
                wall[id] = realloc(wall[id], (step[id]+1)*sizeof(point_t));
                assert(wall[id]);
                wall[id][step[id]] = measurementToPoint(table[i].measurement[j]);
                step[id]++;
            }
        }
        free(table);
        table = NULL;
        
        
        for(int i =0; i <NUMBER_OF_WALLS;i++){
            computeCoefDirecteur(wall[i], step[i], coefsWall[i]);
        }
        for(int i =0; i <NUMBER_OF_WALLS;i++){
            free(wall[i]);
            wall[i] = NULL;}
    }

    point_t intersection[NUMBER_OF_WALLS-1];
    for(int i=1;i < NUMBER_OF_WALLS;i++){
        intersection[i-1].x = (coefsWall[i][1] - coefsWall[i-1][1]) /(coefsWall[i-1][0] - coefsWall[i][0]);
        intersection[i-1].y =  coefsWall[i][0] * intersection[i-1].x + coefsWall[i][1];
    }
    float thetaWall = (-atan(coefsWall[1][0]));

    moveOrigin(intersection[0], thetaWall);
    
    wall.x0 = 0;
    wall.y1 = 0;
    wall.x2 = robotActualPosition.x + cos(thetaWall)*intersection[1].x - sin(thetaWall)*intersection[1].y;
    wall.y3 = robotActualPosition.y + sin(thetaWall)*intersection[2].x + cos(thetaWall)*intersection[2].y;
    
    char toSend[50];
    sprintf(toSend, "Wall location: %d, %d, %d, %d", wall.x0, wall.y1, wall.x2, wall.y3);
    mod_com_writeMessage(toSend, 3);

    
    return true;
    
}


robotDistance_t mod_mapping_getRobotDisplacement(const point_t * newAbsolutePosition){
    robotDistance_t displacement;
    
    int deltaX = newAbsolutePosition->x - robotActualPosition.x;
    int deltaY = newAbsolutePosition->y - robotActualPosition.y;
    float movementAngle = atan(deltaY/deltaX);
    
    char toSend[50];
    sprintf(toSend, "deltaX: %d, deltaY; %d angle: %f",  deltaX,  deltaY, movementAngle);
    mod_com_writeMessage(toSend, 3);
    
    //displacement.rotation= acos((deltaX*cos(robotActualPosition.theta+M_PI/2) + deltaY*sin(robotActualPosition.theta+M_PI/2))/sqrt(deltaX*deltaX+deltaY*deltaY));
    //if((robotActualPosition.theta+M_PI/2 < movementAngle && M_PI + robotActualPosition.theta+M_PI/2 > movementAngle))
       // displacement.rotation = -displacement.rotation;

    float angle;
    
    if(deltaX >= 0 ){
        angle = 3*M_PI/2 - robotActualPosition.theta + atan(deltaY / deltaX);
    }
    else{
        angle = M_PI/2 - robotActualPosition.theta + atan(deltaY / deltaX);
    }

    displacement.rotation = angle;
    
    displacement.translation = sqrt(deltaX*deltaX + deltaY*deltaY);

    while(displacement.rotation > M_PI) displacement.rotation -= 2*M_PI;
    while(displacement.rotation < -M_PI) displacement.rotation += 2*M_PI;
    
    sprintf(toSend, "To do: %d, %f",  displacement.translation,  displacement.rotation);
    mod_com_writeMessage(toSend, 3);

    return displacement;
}


robotPosition_t mod_mapping_getActualPosition(void){
    return robotActualPosition;
}


float mod_mapping_getRelativeAngle(const float angle){
    float newAngle = angle - robotActualPosition.theta - M_PI/2;
    
    while(newAngle > M_PI) newAngle -= 2*M_PI;
    while(newAngle < -M_PI) newAngle += 2*M_PI;
    
    return newAngle;
}


void mod_mapping_checkEnvironment(measurement_t * measurement, int numberOfMeasurements){
    point_t * point = malloc(numberOfMeasurements*sizeof(point_t));
    assert(point);
    environment.numberOfknownObjects = 0;
    environment.numberOfnewObjects = 0;
    
    for(int i=0; i< NUMBER_OF_WALLS; i++){
        environment.nearWall[i] = false;
    }
    
    for(int i=0; i< numberOfMeasurements; i++){
        if(measurement[i].value < 1) continue;
        point[i] = measurementToPoint(&measurement[i]);
        
        if(!isNear(point[i])) continue;
        
        if(point[i].x < wall.x0 + TOLERANCE_WALL) environment.nearWall[0] = true;
        else if(point[i].x > wall.x2 - TOLERANCE_WALL) environment.nearWall[1] = true;
        else if(point[i].y < wall.y1 + TOLERANCE_WALL) environment.nearWall[2] = true;
        else if(point[i].y > wall.y3 - TOLERANCE_WALL) environment.nearWall[3] = true;
        
        else{
            if(checkIfObjectExists(point[i])){
                environment.knownObjectsLocation[environment.numberOfknownObjects] = point[i];
                environment.numberOfknownObjects++;
            }
            else{
                environment.newObjectsLocation[environment.numberOfnewObjects] = point[i];
                environment.numberOfnewObjects++;
                char toSend[50];
                sprintf(toSend, "New object found: %d, %d", point[i].x, point[i].y);
                mod_com_writeMessage(toSend, 3);
                
                objectList = realloc(objectList, sizeof(point_t)*(objectListSize+1));
                objectList[objectListSize] = point[i];
                objectListSize++;
            }
            if((environment.numberOfnewObjects == 3) || (environment.numberOfknownObjects == 3)){
                break;
            }
        }
        
    }
    
    free(point);
    point = NULL;
    
}


robotDistance_t mod_mapping_computeDistanceForPicture(point_t point){
    robotDistance_t toDo = mod_mapping_getRobotDisplacement(&point);
    toDo.translation -= PICTURE_DISTANCE + TOF_RADIUS;
    
    char toSend[50];
    sprintf(toSend, "To do2: %d, %f",  toDo.translation,  toDo.rotation);
    mod_com_writeMessage(toSend, 3);
    
    return toDo;
}


point_t mod_mapping_getAreaCenter(void){
    return (point_t) {wall.x2/2, wall.y3/2};
}


point_t mod_mapping_checkEnvironmentRobotReferencial(measurement_t * measurement, bool considerWalls){
    static int lastStepObjectDistance = 1000;
    point_t point = measurementToPoint(measurement);
    int distance = measurement->value + TOF_RADIUS;
    if(considerWalls){
        if(point.x < wall.x0 + TOLERANCE_WALL || point.x > wall.x2 - TOLERANCE_WALL ||
           point.y < wall.y1 + TOLERANCE_WALL || point.y > wall.y3 - TOLERANCE_WALL){
            return (point_t){-1,-1};
        }
        if(checkIfObjectExists(point)){
            return (point_t){-1,-1};
        }
        else{
            environment.newObjectsLocation[environment.numberOfnewObjects] = point;
            environment.numberOfnewObjects++;
        }
    }
    else{
        if(distance > 300 ||(distance < lastStepObjectDistance + TOLERANCE_OBJECT_BIS && distance > lastStepObjectDistance - TOLERANCE_OBJECT_BIS)){
            lastStepObjectDistance = 1000;
            return (point_t){-1,-1};
        }
        lastStepObjectDistance = distance;
    }
    return point;
}

int mod_mapping_computeDistanceForPictureRobotReferencial(measurement_t * measurement){
    return measurement->value - PICTURE_DISTANCE;
    
}
