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


typedef struct  {
    int left;
    int right;
}wheelSpeed_t;

typedef struct  {
    int direction;
    int mainSpeed;
}robotSpeed_t;

/**
 * @brief Initialize motors
 */
void mod_motors_init(void);

/**
 * @brief Convert the robot style speed in wheel style speed
 *
 * @param[in] robotSpeed     The speed to convert (direction + general speed)
 *
 * @param[out] Speed of the robot from a wheel point of view
 */
wheelSpeed_t mod_motors_convertRobotSpeedToWheelspeed(robotSpeed_t robotSpeedTemp);

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
