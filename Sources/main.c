/*
 * File : main.c
 * Project : e_puck_project
 * Description : Main file, initial point for the program, performs robot actions by calling modules
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include <main.h>

// Standard headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Epuck/ChibiOS headers
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

// Our headers
#include "mod_errors.h"
#include "mod_basicIO.h"
#include "mod_audio.h"
#include "mod_errors.h"

//Semaphores
BSEMAPHORE_DECL(sem_wip, FALSE);

typedef struct{
    int discovering;
    int exploration;
    int sorting;
} history_t;

history_t history = {0,0,0};


_Bool actionChoice(void){

    switch(mod_audio_processedCommand){
        case CMD_DISCOVERING :
            
            break;
        case CMD_EXPLORATION :
            
            break;
        case CMD_SORTING :
            
            break;
        default :
            error(NO_COMMAND);
    }
    return 0;
}


int main(void)
{

    
    halInit();
    chSysInit();
    mpu_init();

    while (1) {
        _Bool commandOK = 0;
        mod_basicIO_changeRobotState(WAITING);
        while(!commandOK){
            mod_audio_listenForSound();
            chBSemWait(&mod_audio_sem_commandAvailable);
            actionChoice();
        }
        mod_basicIO_changeRobotState(WIP);
        commandOK = chBSemWait(&sem_wip);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}




