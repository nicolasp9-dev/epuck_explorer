/*
 * File : mod_exploration.c
 * Project : e_puck_project
 * Description : Module in charge of exploration / autonomous displacements algorithms
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "mod_exploration.h"


// Our headers
#include "mod_mapping.h"
#include "mod_errors.h"
#include "mod_communication.h"
#include <ch.h>

/********************
 *  Private functions
 */


/********************
 *  Public functions
 */


void mod_explo_init(void){
    mod_mapping_init();
}


void mod_explo_discoverTheAreaOnThread(void){
    mod_com_writeDatas("Here all is alright", "TOUOE", 0);
    chThdSleepMilliseconds(500);
    mod_mapping_doInitialMapping();
}

void mod_explo_explorateTheAreaOnThread(exploration_t type){
    
}

void mod_explo_sortTheAreaOnThread(void){
    
}

void mod_explo_sendTheMap(void){
    
}

void mod_explo_waitUntilEndOfWork(void){
    
}
