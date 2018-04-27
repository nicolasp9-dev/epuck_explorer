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
#include "mod_check.h"


// Frequences for sound detection after FFT
#define FFT_SIZE                1024

#define MIN_VALUE_THRESHOLD     150000

#define MIN_FREQ                30
#define FREQ_CMD_DISCOVERING    33    //400Hz
#define FREQ_CMD_EXPLORATION    39    //500Hz
#define FREQ_CMD_MAPSEND        46    //600HZ
#define FREQ_CMD_SING           52    //700HZ
#define FREQ_CMD_CALIBRATION    59    //800HZ
#define MAX_FREQ                62

#define FREQ_CMD_DISCOVERING_L  (FREQ_CMD_DISCOVERING-1)
#define FREQ_CMD_DISCOVERING_H  (FREQ_CMD_DISCOVERING+1)
#define FREQ_CMD_EXPLORATION_L  (FREQ_CMD_EXPLORATION-1)
#define FREQ_CMD_EXPLORATION_H  (FREQ_CMD_EXPLORATION+1)
#define FREQ_CMD_MAPSEND_L      (FREQ_CMD_MAPSEND-1)
#define FREQ_CMD_MAPSEND_H      (FREQ_CMD_MAPSEND+1)
#define FREQ_CMD_SING_L         (FREQ_CMD_SING-1)
#define FREQ_CMD_SING_H         (FREQ_CMD_SING+1)
#define FREQ_CMD_CALIBRATION_L  (FREQ_CMD_CALIBRATION-1)
#define FREQ_CMD_CALIBRATION_H  (FREQ_CMD_CALIBRATION+1)

/********************
 *  Public variables
 */

BSEMAPHORE_DECL(mod_audio_sem_commandAvailable, TRUE);
bool needAudio;
command_t mod_audio_processedCommand;


/********************
 *  Private variables
 */

static float * micFront_cmplx_input;
static float * micFront_output;
//static int measureNumber;

/********************
 *  Private functions
 */


/********************
 *  Public functions (Informations in header)
 */

void mod_audio_initModule(void){
    micFront_cmplx_input = malloc(sizeof(float)*2 * FFT_SIZE);
    micFront_output = malloc(sizeof(float) * FFT_SIZE);
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
    else if(max_norm_index >= FREQ_CMD_MAPSEND_L && max_norm_index <= FREQ_CMD_MAPSEND_H){
        mod_audio_processedCommand = CMD_MAPSEND;
    }
    else if(max_norm_index >= FREQ_CMD_SING_L && max_norm_index <= FREQ_CMD_SING_H){
        mod_audio_processedCommand = CMD_SING;
    }
    else if(max_norm_index >= FREQ_CMD_CALIBRATION_L && max_norm_index <= FREQ_CMD_CALIBRATION_H){
        mod_audio_processedCommand = CMD_CALIBRATION;
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
    assert(micFront_cmplx_input);
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
    assert(micFront_output);
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
                return;
            }
            
        }
        nb_samples = 0;
        process++;
    }
}



void mod_audio_launchMelodyOnThread(music_t musicName, playMode_t playMode){
    stop_current_melody();
    return;
    // New area
    switch(musicName){
        case MUS_DISCOVERING:
            play_melody(MARIO);
            break;
        case MUS_EXPLORATION:
            play_melody(UNDERWORLD);
            break;
        case MUS_SONG:
            play_melody(STARWARS);
            break;

    }
}


void mod_audio_alertInterruption(alert_t alertName){
    stop_current_melody();
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
