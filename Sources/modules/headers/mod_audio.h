/*
 * File : mod_audio.h
 * Project : e_puck_project
 * Description : Module that contains acquisition, process and diffusion of audio signals
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#ifndef _MOD_AUDIO_
#define _MOD_AUDIO_

#include <ch.h>

/**
 * @brief Melodies names
 */
typedef enum{
    DISCOVERING=0,
    EXPLORATION,
    SORTING
} music_t;

/**
 * @brief Alert names
 */
typedef enum{
    SHORT=0,
    LONG
} alert_t;

/**
 * @brief Possible commands
 */
typedef enum{
    NOTHING=0,
    CMD_DISCOVERING,
    CMD_EXPLORATION,
    CMD_SORTING
} command_t;

/**
 * @brief Public accès to variable in source file
 */
extern binary_semaphore_t mod_audio_sem_commandAvailable;
extern command_t mod_audio_processedCommand;


/**
 * @brief Initialize the audio module, chack if audio device ready
 */
void mod_audio_initModule(void);

/**
 * @brief Play a melody on the robot
 * @details It have a low priority because it's not important
 *
 * @param[in] musicName     The name of the melody to play
 */
void mod_audio_launchMelody(music_t);

/**
 * @brief Play the alert after stoping the current melody (if it's playing)
 *
 * @param[in] alertName     The name of the alert to play
 */
void mod_audio_alertInterruption(alert_t);

/**
 * @brief Interrupt the audio stream currently playing
 */
void mod_audio_interruptMelody(void);

/**
 * @brief The robot start listening for external sounds
 */
void mod_audio_listenForSound(void);

/**
 * @brief The robot stop listening for external sounds
 */
void mod_audio_stopListenForSound(void);


#endif
