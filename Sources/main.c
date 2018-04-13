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
void exploration(void){
    mod_basicIO_changeRobotState(WIP);
    mod_com_writeDatas("EXPLO", "TOUOE", 0);
    if(history.exploration > 0){
        mod_audio_launchMelodyOnThread(EXPLORATION_FAST, REPEAT);
        mod_explo_explorateTheAreaOnThread(CONTINUE);
        mod_explo_waitUntilEndOfWork();
        
    }
    else{
        mod_audio_launchMelodyOnThread(EXPLORATION, REPEAT);
        mod_explo_explorateTheAreaOnThread(NEW);
        mod_explo_waitUntilEndOfWork();
    }
    
    history.exploration +=1;
}


/**
 * @brief Launch the discovering of the area
 *
 * @note It blocks the thread until the end of the action
 */
void discovering(void){
    mod_basicIO_changeRobotState(WIP);
    
    history=  (history_t){0,0,0};
    mod_audio_launchMelodyOnThread(DISCOVERING, REPEAT);
    mod_explo_discoverTheAreaOnThread();
    mod_explo_waitUntilEndOfWork();
    
    history.discovering +=1;
}

/**
 * @brief Launch the sorting on the area
 *
 * @note It blocks the thread until the end of the action
 */
void sorting(void){
    mod_basicIO_changeRobotState(WIP);

    mod_audio_launchMelodyOnThread(SORTING, REPEAT);
    mod_explo_sortTheAreaOnThread();
    mod_explo_waitUntilEndOfWork();
    history.sorting +=1;
}

/**
 * @brief Launch the action that play StarWars song
 *
 * @note It blocks the thread until the end of the action
 */
void song(void){
    mod_basicIO_changeRobotState(WIP);
    
    mod_audio_launchMelodyOnThread(MUSIC, SINGLE);
    mod_audio_waitUntilMelodyEnd();
}

/**
 * @brief Launch the action that send the map to the user
 *
 * @note It blocks the thread until the end of the action
 */
void sendMap(void){
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
            mod_com_writeDatas("DISCOVER", "TOUOE", 0);
            discovering();
            break;
        
        case CMD_EXPLORATION :
            mod_com_writeDatas("EXPLO", "TOUOE", 0);
            if(history.discovering > 0) { exploration(); }
            else{
                
                mod_audio_alertInterruption(IMPOSSIBLE);
                mod_audio_waitUntilMelodyEnd();
                return;
            }
            break;
        
        case CMD_SORTING :
            mod_com_writeDatas("SORT", "TOUOE", 0);
            if(history.discovering > 0 && history.exploration > 0 && history.sorting == 0){ sorting(); }
            else{
                mod_audio_alertInterruption(IMPOSSIBLE);
                mod_audio_waitUntilMelodyEnd();
                return;
            }
            break;
        
        case CMD_MAPSEND :
            mod_com_writeDatas("MAPSEND", "TOUOE", 0);
            if(history.discovering > 0){ sendMap(); }
            else{
                mod_audio_alertInterruption(IMPOSSIBLE);
                mod_audio_waitUntilMelodyEnd();
                return;
            }
            break;
            
        case CMD_SING :
            mod_com_writeDatas("SING", "TOUOE", 0);
            song();
            break;
        
        default :
            mod_audio_alertInterruption(IMPOSSIBLE);
            mod_audio_waitUntilMelodyEnd();
            return;
    }
    mod_com_writeDatas("END OF", "TOUOE", 0);
    mod_audio_alertInterruption(LONG);
    mod_audio_waitUntilMelodyEnd();
}

void initSystem(void){
    halInit();
    chSysInit();
    mod_audio_initModule();
    mod_com_initConnexion();
    mod_explo_init();
}

void calibrateSystem(void){
    mod_com_writeDatas("Info", "The system is strating the calibration...", 0);
    mod_mapping_calibrateTheSystem();
    mod_com_writeDatas("Info", "End of calibration, data saved.", 0);
}

int main(void)
{
    initSystem();
    calibrateSystem();
    
    //mod_audio_listenForSound();
    mod_explo_discoverTheAreaOnThread();
    while (1) {

        /*mod_com_writeDatas("Robot is waiting", "TOUOE", 0);
        needAudio = true;
        chBSemWait(&mod_audio_sem_commandAvailable);
        mod_com_writeDatas("Robot have action to do", "TOUOE", 0);
        actionChoice();
        mod_audio_processedCommand = NOTHING;*/
        
        /*char toSend[100];
        int table[10];
        getAllProximityValues(table);
        char* pointer = toSend;
        int i;
        for(i=0; i<8;i++){
            sprintf(pointer, "%d ",table[i]);
            pointer= toSend + strlen(toSend);
        }
        mod_com_writeDatas(toSend, "TOUOE", 0);
        
        if(testSensor() < 50){
            mod_basicIO_changeRobotState(WIP);
        }
        else{
            mod_basicIO_changeRobotState(WAITING);
            
        }*/
        
        
        chThdSleepMilliseconds(400);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}




