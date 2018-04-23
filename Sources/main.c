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
#include "mod_exploration.h"
#include "mod_communication.h"

// Temporary
#include "mod_mapping.h"

// Semaphores
BSEMAPHORE_DECL(sem_wip, FALSE);

typedef struct{
    int discovering;
    int exploration;
    int sorting;
} history_t;

history_t history = {0,0,0};


void initSystem(void){
    halInit();
    chSysInit();
    
    mod_audio_initModule();
    mod_com_initModule();
    mod_explo_initModule();
}

void exploration(void){
    mod_basicIO_changeRobotState(WIP);
    mod_com_writeMessage("Will explore the area", 3);
    
    if(history.exploration > 0){
        mod_audio_launchMelodyOnThread(EXPLORATION_FAST, REPEAT);
        mod_explo_explorateTheAreaOnThread(IMPROVE);
        mod_explo_waitUntilEndOfWork();
        
    }
    else{
        mod_audio_launchMelodyOnThread(EXPLORATION, REPEAT);
        mod_explo_explorateTheAreaOnThread(NEW);
        mod_explo_waitUntilEndOfWork();
    }
    
    history.exploration +=1;
}

void discovering(void){
    mod_basicIO_changeRobotState(WIP);
    mod_com_writeMessage("Will discover the area", 3);
    
    history=  (history_t){0,0,0};
    mod_audio_launchMelodyOnThread(DISCOVERING, REPEAT);
    mod_explo_discoverTheAreaOnThread();
    mod_explo_waitUntilEndOfWork();
    
    history.discovering +=1;
}

void song(void){
    mod_basicIO_changeRobotState(WIP);
    mod_com_writeMessage("Will launch a melody", 3);
    
    mod_audio_launchMelodyOnThread(MUSIC, SINGLE);
    mod_audio_waitUntilMelodyEnd();
}

void sendMap(void){
    mod_basicIO_changeRobotState(WIP);
    mod_com_writeMessage("Will send the map", 3);
    
    mod_explo_sendTheMap();
}

void calibrateSystem(void){
    mod_basicIO_changeRobotState(WIP);
    mod_com_writeMessage("Will calibrate the system", 3);
    
    mod_explo_calibration();
}

void actionChoice(void){
    switch(mod_audio_processedCommand){
        case CMD_DISCOVERING :
            discovering();
            break;
        
        case CMD_EXPLORATION :
            if(history.discovering > 0) { exploration(); }
            else{
                mod_audio_alertInterruption(IMPOSSIBLE);
                mod_audio_waitUntilMelodyEnd();
                return;
            }
            break;
        
        case CMD_MAPSEND :
            if(history.discovering > 0){ sendMap(); }
            else{
                mod_audio_alertInterruption(IMPOSSIBLE);
                mod_audio_waitUntilMelodyEnd();
                return;
            }
            break;
            
        case CMD_SING :
            song();
            break;
        
        default :
            mod_audio_alertInterruption(IMPOSSIBLE);
            mod_audio_waitUntilMelodyEnd();
            return;
    }
    
    mod_audio_alertInterruption(LONG);
    mod_audio_waitUntilMelodyEnd();
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}


int main(void)
{
    initSystem();
    
    mod_com_writeMessage("Basic init OK", 4);
    
    chThdSleepMilliseconds(600);
    //mod_audio_listenForSound();
    discovering();
    while (1) {
        mod_com_writeMessage("Robot is waiting", 3);
        needAudio = true;
        chBSemWait(&mod_audio_sem_commandAvailable);
        mod_com_writeMessage("Signal was detected", 3);
        actionChoice();
        mod_audio_processedCommand = NOTHING;
        
        
        chThdSleepMilliseconds(1500);
    }
}





