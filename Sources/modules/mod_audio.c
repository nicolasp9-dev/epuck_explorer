/*
 * File : mod_audio.c
 * Project : e_puck_project
 * Description : Module that contains acquisition, process and diffusion of audio signals
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "mod_audio.h"

// Standard headers
#include <stdio.h>

// Epuck/ChibiOS headers
#include <ch.h>
#include "audio/play_melody.h"
#include "audio/microphone.h"
#include "fft.h"
#include <arm_math.h>

// Our headers
#include "mod_errors.h"
#include "mod_communication.h"



// Frequences for sound detection after FFT
#define FFT_SIZE                1024

#define MIN_VALUE_THRESHOLD     7000

#define MIN_FREQ                10    
#define FREQ_CMD_DISCOVERING    16    //250Hz
#define FREQ_CMD_EXPLORATION    19    //296Hz
#define FREQ_CMD_SORTING        23    //359HZ
#define FREQ_CMD_MAPSEND        26    //406HZ
#define FREQ_CMD_SING           29    //452HZ
#define MAX_FREQ                32

#define FREQ_CMD_DISCOVERING_L  (FREQ_CMD_DISCOVERING-1)
#define FREQ_CMD_DISCOVERING_H  (FREQ_CMD_DISCOVERING+1)
#define FREQ_CMD_EXPLORATION_L  (FREQ_CMD_EXPLORATION-1)
#define FREQ_CMD_EXPLORATION_H  (FREQ_CMD_EXPLORATION+1)
#define FREQ_CMD_SORTING_L      (FREQ_CMD_SORTING-1)
#define FREQ_CMD_SORTING_H      (FREQ_CMD_SORTING+1)
#define FREQ_CMD_MAPSEND_L      (FREQ_CMD_MAPSEND-1)
#define FREQ_CMD_MAPSEND_H      (FREQ_CMD_MAPSEND+1)
#define FREQ_CMD_SING_L         (FREQ_CMD_SING-1)
#define FREQ_CMD_SING_H         (FREQ_CMD_SING+1)

/********************
 *  Public variables
 */

BSEMAPHORE_DECL(mod_audio_sem_commandAvailable, TRUE);
bool needAudio;
command_t mod_audio_processedCommand;


/********************
 *  Private variables
 */

float micFront_cmplx_input[2 * FFT_SIZE];
float micFront_output[FFT_SIZE];


/********************
 *  Private functions
 */


/********************
 *  Public functions (Informations in header)
 */

void mod_audio_initModule(void){
    needAudio = false;
    play_melody_start();
}



/**
 * @brief Function used to detect the highest value
 *
 * @note Inspired from processAudioData function of TP5
 */
void action_detection(float* data){
    float max_norm = MIN_VALUE_THRESHOLD;
    int16_t max_norm_index = -1;
    
    
    //search for the highest peak
    for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
        if(data[i] > max_norm){
            max_norm = data[i];
            max_norm_index = i;
        }
    }
    if(max_norm_index >= FREQ_CMD_DISCOVERING_L && max_norm_index <= FREQ_CMD_DISCOVERING_H){
        mod_audio_processedCommand = CMD_DISCOVERING;
    }

    else if(max_norm_index >= FREQ_CMD_EXPLORATION_L && max_norm_index <= FREQ_CMD_EXPLORATION_H){
        mod_audio_processedCommand = CMD_EXPLORATION;
    }

    else if(max_norm_index >= FREQ_CMD_SORTING_L && max_norm_index <= FREQ_CMD_SORTING_H){
        mod_audio_processedCommand = CMD_SORTING;
    }
    else if(max_norm_index >= FREQ_CMD_MAPSEND_L && max_norm_index <= FREQ_CMD_MAPSEND_H){
        mod_audio_processedCommand = CMD_MAPSEND;
    }
    else if(max_norm_index >= FREQ_CMD_SING_L && max_norm_index <= FREQ_CMD_SING_H){
        mod_audio_processedCommand = CMD_SING;
    }
    else{
        mod_audio_processedCommand = NOTHING;
    }
}


/**
 * @brief Callback that process audio datas to extract desired command
 *
 * @note Inspired from processAudioData function of TP5
 */
void processDatas(int16_t *data, uint16_t num_samples){
    if(needAudio == false){
        return;
    }
    static uint16_t nb_samples = 0;
    static uint8_t process = 0;

    // Get samples in a complex array
    for(uint16_t i = 0 ; i < num_samples ; i+=4){
        micFront_cmplx_input[nb_samples] = (float)data[i+3];
        
        nb_samples++;
        micFront_cmplx_input[nb_samples] = 0;
        nb_samples++;
        
        //stop storing when buffer is full
        if(nb_samples >= (2 * FFT_SIZE)){
            break;
        }
    }
    chThdSleepMilliseconds(10);
    
    // Process when buffer is full
    if(nb_samples >= (2 * FFT_SIZE)){
        // FFT Processing
        doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

        // Magnitude processing
        arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
        if(process > 8){
            action_detection(micFront_output);
            process = 0;
            if(mod_audio_processedCommand != NOTHING){
                needAudio = false;
                nb_samples = 0;
                chBSemSignal(&mod_audio_sem_commandAvailable);
                mod_com_writeDatas("Action processed", "TOUOE", 0);
                return;
            }
            
        }
        nb_samples = 0;
        process++;
    }
}



void mod_audio_launchMelodyOnThread(music_t musicName, playMode_t playMode){
    mod_com_writeDatas("mod_audio_launchMelodyOnThread before stop", "TOUOE", 0);
    stop_current_melody();
    mod_com_writeDatas("mod_audio_launchMelodyOnThread after stop", "TOUOE", 0);

    // New area
    switch(musicName){
        case DISCOVERING:
            play_melody(MARIO);
            break;
            
        case EXPLORATION:
            play_melody(UNDERWORLD);
            break;
            
        case EXPLORATION_FAST:
            play_melody(MARIO);
            break;
            
        case MUSIC:
            play_melody(STARWARS);
            break;
            
        case SORTING:
            play_melody(MARIO);
            break;
    }
}


void mod_audio_alertInterruption(alert_t alertName){
    mod_com_writeDatas("mod_audio_alertInterruption before stop", "TOUOE", 0);
    stop_current_melody();
    mod_com_writeDatas("mod_audio_alertInterruption after stop", "TOUOE", 0);
    switch(alertName){
        case SHORT:
            play_melody(ALERT);
            break;
            
        case LONG:
            play_melody(ALERT);
            break;
            
        case IMPOSSIBLE:
            play_melody(ALERT);
            break;
    }
    
}

void mod_audio_listenForSound(void){
    mic_start(&processDatas);
}


void  mod_audio_waitUntilMelodyEnd(void){
    wait_until_melody_end();
}
