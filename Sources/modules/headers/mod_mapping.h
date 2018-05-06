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
#define NUMBER_OF_STEPS             30
#define ANGLE_ELEMENT               2*M_PI/NUMBER_OF_STEPS
#define COMPLETE_ANGLE              2*M_PI

#define NUMBER_OF_STEPS_SCAN360     50
#define ANGLE_ELEMENT_SCAN360       2*M_PI/NUMBER_OF_STEPS_SCAN360

#define NUMBER_OF_STEPS_FRONT       10
#define ANGLE_ELEMENT_FRONT         2*M_PI/(16*NUMBER_OF_STEPS_FRONT)
#define SIZE_FRONT_SCAN             2*M_PI/16

#define NUMBER_OF_SCANS_MIN         NUMBER_OF_STEPS_SCAN360

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
    float rotation;
}robotDistance_t;

typedef struct{
    bool nearWall[4];
    int numberOfnewObjects;
    int numberOfknownObjects;
    point_t newObjectsLocation[3];
    point_t knownObjectsLocation[3];
} actualEnvironement_t;

extern actualEnvironement_t environment;

/**
 * @brief Init needed libraries by mapping + Change position of the robot to the origin
*/
void mod_mapping_init(void);

/**
 * @brief Change the position of the robot to the origin
 */
void mod_mapping_resetCoordinates(void);

/**
 * @brief Compute the position of the robot based on last position and the displacement (speed + time)
 *
 * @param[in] wheelSpeed       The wheel style speed of the robot
 * @param[in] time             The time the robot was at the indicated speed
*/
void mod_mapping_updatePositionWheelSpeedType(const wheelSpeed_t *wheelSpeed, int time);


/**
 * @brief Compute the wall location based on measurements and store information in the mapping area
 *
 * @param[in] measurement       The measurement of all walls points
 */
_Bool mod_mapping_computeWallLocation(measurement_t* measurement);

/**
 * @brief Compute the wall location based on measurements and store information in the mapping area
 *
 * @param[in] newAbsolutePosition       The point where to go
 *
 * @param[out]        The absolute path to the point (Rotation + Translation)
 */
robotDistance_t mod_mapping_getRobotDisplacement(const point_t * newAbsolutePosition);

/**
 * @brief Returns the actual position in the main coordinates system of the robot
 *
 * @param[out]        The position
 */
robotPosition_t mod_mapping_getActualPosition(void);

/**
 * @brief Returns the relative to robot angle of the angle
 *
 * @param[in] angle       The relative angle
 *
 * @param[out]      The absolute angle
 */
float mod_mapping_getRelativeAngle(const float angle);

/**
 * @brief Check the current environement in front of the robot
 *
 * @param[in] measurement               All measurements
 * @param[in] numberOfMeasurements      The number of measurements
 * @param[in] environmentObstacles      The environement content
 */
void mod_mapping_checkEnvironment(measurement_t * measurement, int numberOfMeasurements);

/**
 * @brief Returns the path to do to take the picture
 *
 * @param[in] point     The point to take in piicture
 *
 * @param[out]      The path to do
 */
robotDistance_t mod_mapping_computeDistanceForPicture(point_t point);


/**
 * @brief Returns the point of the center of the arae
 *
 * @param[out]      The point of the center
 */
point_t mod_mapping_getAreaCenter(void);


/**
 * @brief Check if the object exists
 *
 * @param[in] measurement     The measruement of the object
 * @param[in] considerWalls   True if discovering have been done, so it will use points
 *
 * @param[out]      The location of the point (Robot or fix referencial)
 */

point_t mod_mapping_checkEnvironmentRobotReferencial(measurement_t * measurement, bool considerWalls);

bool mod_mapping_checkEnvironmentLimitsRobotReferencial(measurement_t * measurement, bool considerWalls);

/**
 * @brief Returns the path to do to take the picture, without using the referencial but only distance datas in the mesaurement
 *
 * @param[in] measurement     The measruement of the object
 *
 * @param[out]      The path to do (No angle, straight line)
 */
int mod_mapping_computeDistanceForPictureRobotReferencial(measurement_t * measurement);

/**
 * @brief Find the best position where to move to have the center of the object
 *
 * @param[in] measurement     The list of measruements
 * @param[in] considerWalls   True if discovering have been done, so it will use points
 * @param[in] newPoint        The closest point
 *
 * @param[out]      The distance to do to take the picture
 */
robotDistance_t mod_mapping_findObjectBestPosition(measurement_t* measurement, point_t* newPoint, bool considerWalls);


#endif
