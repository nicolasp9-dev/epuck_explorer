/*
 * File : mod_motors.h
 * Project : e_puck_project
 * Description : Module that manages actuators
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */



#ifndef _MOD_MOTORS_
#define _MOD_MOTORS_

#define ROBOT_RADIUS    27
#define MS_TO_S         1000
#include "structs.h"
/**
 * @brief Initialize motors
 */
void mod_motors_init(void);

/**
 * @brief Initialize static values for a good calibration
 */
void mod_motors_initCalibration(void);

/**
 * @brief Convert the robot style speed in wheel style speed
 *
 * @param[in] robotSpeed     The speed to convert (direction + general speed)
 *
 * @param[out] Speed of the robot from a wheel point of view
 */
wheelSpeed_t mod_motors_convertRobotSpeedToWheelspeed(robotSpeed_t robotSpeedTemp);

/**
 * @brief Convert the wheel style speed in robot style speed
 *
 * @param[in] wheelSpeed     The speed to convert (wheel left / wheel right)
 *
 * @param[out] Speed of the wheels from a robot point of view
 */
robotSpeed_t mod_motors_convertWheelSpeedToMotorspeed(wheelSpeed_t wheelSpeedTemp);

/**
 * @brief Based on data for a complete rotation, the robot compute constants for motors
 *
 * @param[in] displacement  The displacement speed applied to the robot
 * @param[in] movementTime  The time it took to do a complete rotation
 */
void mod_motor_angleCalibration(const wheelSpeed_t displacement, const int movementTime);

/**
 * @brief Based on a displacement, the robot compute constants for motors
 * @note This function must be called to time, one before the displacement, and the second after it
 *
 * @param[in] displacement      The displacement speed applied to the robot
 * @param[in] movementTime      The time this speed was applied
 * @param[in] distanceWithWall  The distance with the wall
 */
void mod_motor_distanceCalibration(int distanceWithWall, const wheelSpeed_t displacement, const int movementTime);


/**
 * @brief Change Motors command for a certain state
 *
 * @param[in] wheelSpeed     Desired speed for each wheel
 */
void mod_motors_changeStateWheelSpeedType(wheelSpeed_t wheelSpeed);

/**
 * @brief Change Motors command for a certain state
 *
 * @param[in] wheelSpeed     Desired speed in robot speed style struct
 */
void mod_motors_changeStateRobotSpeedType(robotSpeed_t robotSpeed);


/**
 * @brief Stop the robot
 */
void mod_motors_stop(void);


#endif
