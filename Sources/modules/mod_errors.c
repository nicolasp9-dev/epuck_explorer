/*
 * File : errors.c
 * Project : e_puck_project
 * Description : Module in charge of giving errors feedback
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include <ch.h>

#include "headers/mod_errors.h"
#include "headers/mod_basicIO.h"

void error(error_t type){
    while(1){
        mod_basicIO_alert_leds(type);
        chThdSleepMilliseconds(2000);
    }
    
}
