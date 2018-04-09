/*
 * File : mod_basicIO.h
 * Project : e_puck_project
 * Description : Module that manages signals for basic inputs/outputs (buttons/leds)
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#ifndef _MOD_BASICIO_
#define _MOD_BASICIO_

/**
 * @brief Possible states of the robot
 */
typedef enum{
    WIP = 0,
    WAITING
}state_t;

/**
 * @brief Function that switch LEDs to inform the user of the robot state
 *
 * @param[in] state     The current state of the robot
 */
void mod_basicIO_changeRobotState(state_t state);


/**
 * @brief Function that switch all LEDs off and blink the front red led for the corresponding number of blink
 *
 * @param[in] numberofblinking     The number of blink
 */
void mod_basicIO_alert_leds(int numberofblinking);

#endif
