/*
 * File : mod_audio.c
 * Project : e_puck_project
 * Description : Module that contains acquisition, process and diffusion of audio signals
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "headers/mod_audio.h"
#include "audio/play_melody.h"
#include "ch.h"

void mod_audio_emitSound(void){
    play_melody_start();
    chThdSleepMilliseconds(500);
    play_note(NOTE_B0, 1000);
}
