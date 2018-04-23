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

#define ANGLE_ELEMENT               20
#define COMPLETE_ANGLE              360
#define NUMBER_OF_STEPS             (COMPLETE_ANGLE/ANGLE_ELEMENT)
#include "structs.h"

typedef struct {
    int x;
    int y;
    float theta;
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
    float rotation[2];
}robotDistance_t;

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
void mod_mapping_computeWallLocation(measurement_t* measurement, int number);

robotDistance_t mod_mapping_getRobotDisplacement(const robotPosition_t * newAbsolutePosition);

robotPosition_t mod_mapping_getActualPosition(void);
#endif
