/*
 * File : mod_motors.c
 * Project : e_puck_project
 * Description : Module that manages actuators
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "mod_motors.h"
#include "motors.h"
#include "mod_mapping.h"
#include <arm_math.h>


void mod_motors_init(void){
    motors_init();
    // + Save initial position
}

void mod_motors_state(int direction, int speed){
    if(direction > 1000)
        direction = 1000;
    else if(direction < -1000)
        direction = -1000;
    
    if(speed > 1100)
        speed = 1100;
    else if(speed < -1100)
        speed = -1100;
    
    left_motor_set_speed(-speed * (direction-1000)/1000);
    right_motor_set_speed(speed * (direction+1000)/1000);
    
}

void mod_motors_stop(void){
    mod_motors_state(.0, .0);
}
