/*
 * File : mod_basicIO.c
 * Project : e_puck_project
 * Description : Module that manages signals for basic inputs/outputs (buttons/leds)
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "headers/mod_basicIO.h"

// Epuck/ChibiOS headers
#include "leds.h"


#define ON              1
#define OFF             0

#define ERROR_TIME_ON   500
#define ERROR_TIME_OFF  300

/********************
 *  Public functions (Informations in header)
 */

void mod_basicIO_changeRobotState(state_t state){
    switch(state){
        case WIP:
            set_body_led(OFF);
            set_front_led(ON);
            break;
        case WAITING:
            set_body_led(ON);
            set_front_led(OFF);
            break;
    }
}

void mod_basicIO_alert_leds(int numberofblinking){
    clear_leds();
    int i = 0;
    for(i=1; i <=numberofblinking;i++){
        set_front_led(OFF);
        chThdSleepMilliseconds(ERROR_TIME_OFF);
        set_front_led(ON);
        chThdSleepMilliseconds(ERROR_TIME_ON);
    }
    set_front_led(OFF);
}
