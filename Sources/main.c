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
#include "mod_exploration.h"


//Semaphores
BSEMAPHORE_DECL(sem_wip, FALSE);

typedef struct{
    int discovering;
    int exploration;
    int sorting;
} history_t;

history_t history = {0,0,0};


/**
 * @brief Launch the exploration of the area
 *
 * @note It blocks the thread until the end of the action
 */
void exploration(){
    if(history.exploration > 0){
        mod_audio_launchMelodyOnThread(EXPLORATION_FAST, REPEAT);
        mod_explo_explorateTheAreaOnThread(CONTINUE);
        mod_explo_waitUntilEndOfWork();
        mod_audio_interruptMelody();
    }
    else{
        mod_audio_launchMelodyOnThread(EXPLORATION, REPEAT);
        mod_explo_explorateTheAreaOnThread(NEW);
        mod_explo_waitUntilEndOfWork();
        mod_audio_interruptMelody();
    }
    
    history.exploration +=1;
}


/**
 * @brief Launch the discovering of the area
 *
 * @note It blocks the thread until the end of the action
 */
void discovering(){
    history = {0,0,0};
    mod_audio_launchMelodyOnThread(DISCOVERING, REPEAT);
    mod_explo_discoverTheAreaOnThread();
    mod_explo_waitUntilEndOfWork();
    mod_audio_interruptMelody();
    
    history.discovering +=1;
}

/**
 * @brief Launch the sorting on the area
 *
 * @note It blocks the thread until the end of the action
 */
void sorting(){
    mod_audio_launchMelodyOnThread(SORTING, REPEAT);
    mod_explo_sortTheAreaOnThread();
    mod_explo_waitUntilEndOfWork();
    mod_audio_interruptMelody();
    history.sorting +=1;
}

/**
 * @brief Launch the action that play StarWars song
 *
 * @note It blocks the thread until the end of the action
 */
void song(){
    mod_audio_launchMelodyOnThread(STARWARS);
    mod_audio_waitUntilMelodyEnd();
}

/**
 * @brief Launch the action that send the map to the user
 *
 * @note It blocks the thread until the end of the action
 */
void sendMap(){
    mod_explo_sendTheMap();
}

/**
 * @brief Launch the action to do based on audio feedback
 *
 * @note The robot emit a different audio signal if the action is not possible (dependent on a previous one)
 */
void actionChoice(void){

    switch(mod_audio_processedCommand){
        case CMD_DISCOVERING :
            
            discovering();
            break;
        
        case CMD_EXPLORATION :
            
            if(history.discovering > 0) { exploration(); }
            else{ mod_audio_alertInterruption(IMPOSSIBLE); return; }
            break;
        
        case CMD_SORTING :
            
            if(history.discovering > 0 && history.exploration > 0 && history.sorting == 0){ sorting(); }
            else{ mod_audio_alertInterruption(IMPOSSIBLE); return; }
            break;
        
        case CMD_MAPSEND :
            
            if(history.discovering > 0){ sendMap(); }
            else{ mod_audio_alertInterruption(IMPOSSIBLE); return; }
            break;
            
        case CMD_SING :
            
            song();
            break;
        
        default :
            mod_audio_alertInterruption(IMPOSSIBLE);
            //error(NO_COMMAND);
            return;
    }
    
    mod_audio_alertInterruption(LONG);
}


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    mod_audio_initModule();

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




