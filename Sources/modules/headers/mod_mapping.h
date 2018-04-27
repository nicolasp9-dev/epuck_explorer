/*
 * File : mod_mapping.h
 * Project : e_puck_project
 * Description : Module that compute and store datas of the domain
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_MAPPING_
#define _MOD_MAPPING_

#include "math.h"
#include "stdbool.h"
#define NUMBER_OF_STEPS             32
#define ANGLE_ELEMENT               2*M_PI/NUMBER_OF_STEPS
#define COMPLETE_ANGLE              2*M_PI

#define NUMBER_OF_STEPS_FRONT       20
#define ANGLE_ELEMENT_FRONT         M_PI/2*NUMBER_OF_STEPS_FRONT

#include "structs.h"

typedef struct {
    int x;
    int y;
    double theta;
}robotPosition_t;

typedef struct {
    int x;
    int y;
}point_t;

typedef struct  {
    robotPosition_t position;
    int value;
}measurement_t;

typedef struct  {
    int translation;
    double rotation[2];
}robotDistance_t;


typedef struct{
    bool nearWall[4];
    int numberOfnewObjects;
    int numberOfknownObjects;
    point_t newObjectsLocation[3];
    point_t knownObjectsLocation[3];
} actualEnvironement_t;


/**
 * @brief Init needed libraries by mapping + Change position of the robot to the origin
*/
void mod_mapping_init(void);

/**
 * @brief Change the position of the robot to the origin
 */
void mod_mapping_resetCoordinates(void);

/**
 * @brief Change the position to another landmark (Shift)
 */
void mod_mapping_adaptCoordinatesToOrigin(int x, int y, int theta);

/**
 * @brief Compute the position of the robot based on last position and the displacement (speed + time)
 *
 * @param[in] wheelSpeed       The wheel style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
*/
void mod_mapping_updatePositionWheelSpeedType(const wheelSpeed_t *wheelSpeed, int time);

/**
 * @brief Compute the wall location based on measurements and store information in the mapping area
 */
_Bool mod_mapping_computeWallLocation(measurement_t* measurement, int number);

robotDistance_t mod_mapping_getRobotDisplacement(const robotPosition_t * newAbsolutePosition);

robotPosition_t mod_mapping_getActualPosition(void);

double mod_mapping_getAngleForTranslation(const double angle);

void mod_map_saveWall(measurement_t value, int wallNum);

void mod_mapping_checkEnvironment(measurement_t * measurement, int numberOfMeasurements, actualEnvironement_t * environmentObstacles);
#endif
