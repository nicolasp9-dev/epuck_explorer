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


/**
 * @brief Change Motors command for a certain state
 *
 * @param[in] direction     From -1000 to 1000 (totally left to totally right)
 * @param[in] speed         From -1100 to 1100
 */
void mod_motors_state(int direction, int speed);

/**
 * @brief Initialize motors
 */
void mod_motors_init(void);

/**
 * @brief Stop the robot
 */
void mod_motors_stop(void);


#endif
