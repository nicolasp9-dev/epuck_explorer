/*
 * File : mod_basicIO.c
 * Project : e_puck_project
 * Description : Module that manages signals for basic inputs/outputs (buttons/leds)
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "headers/mod_basicIO.h"
#include "leds.h"

#define ON  1
#define OFF 0

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
        set_front_led(0);
        chThdSleepMilliseconds(300);
        set_front_led(1);
        chThdSleepMilliseconds(500);
    }
    set_front_led(0);
}
