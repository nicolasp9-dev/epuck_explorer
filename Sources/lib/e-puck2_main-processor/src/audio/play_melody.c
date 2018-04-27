/*

File    : play_melody.c
Author  : Eliot Ferragni
Date    : 4 january 2018
REV 1.0

Functions and defines to play little melodies on the speaker

Adapted from the code written by Dipto Pratyaksa
taken at https://www.princetronics.com/supermariothemesong/
*/

/*
 WARNING : Edited by Maxime Marchionno and Nicolas Peslerbe.
 */


#include "play_melody.h"
#include "audio_thread.h"
#include <ch.h>

typedef struct{
	uint16_t* notes;
	float* tempo;
	uint16_t length;
}melody_t;

static thread_reference_t play_melody_ref = NULL;
static bool play = true;
BSEMAPHORE_DECL(endPlaying_sem, true);

//Mario main theme melody
static uint16_t mario_melody[] = {
  NOTE_E5, NOTE_E5, 0, NOTE_E5,
  0, NOTE_C5, NOTE_E5, 0,
  NOTE_G5, 0, 0, 0,
  NOTE_G4, 0, 0, 0,
 
  NOTE_C5, 0, 0, NOTE_G4,
  0, 0, NOTE_E4, 0,
  0, NOTE_A4, 0, NOTE_B4,
  0, NOTE_AS4, NOTE_A4, 0,
 
  NOTE_G4, NOTE_E5, NOTE_G5,
  NOTE_A5, 0, NOTE_F5, NOTE_G5,
  0, NOTE_E5, 0, NOTE_C5,
  NOTE_D5, NOTE_B4, 0, 0,
 
  NOTE_C5, 0, 0, NOTE_G4,
  0, 0, NOTE_E4, 0,
  0, NOTE_A4, 0, NOTE_B4,
  0, NOTE_AS4, NOTE_A4, 0,
 
  NOTE_G4, NOTE_E5, NOTE_G5,
  NOTE_A5, 0, NOTE_F5, NOTE_G5,
  0, NOTE_E5, 0, NOTE_C5,
  NOTE_D5, NOTE_B4, 0, 0
};
//Mario main them tempo
static float mario_tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};
//Underworld melody
static uint16_t underworld_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_D4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};
//Underwolrd tempo
static float underworld_tempo[] = {
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  6, 18, 18, 18,
  6, 6,
  6, 6,
  6, 6,
  18, 18, 18, 18, 18, 18,
  10, 10, 10,
  10, 10, 10,
  3, 3, 3
};

//Star Wars melody
static uint16_t starwars_melody[] = {
	NOTE_A4, NOTE_A4, NOTE_A4,
	NOTE_F4, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4, 
	0,
	NOTE_E5, NOTE_E5, NOTE_E5,
	NOTE_F5, NOTE_C5, NOTE_GS4, NOTE_F4, NOTE_C5, NOTE_A4,
};

//Star Wars tempo
static float starwars_tempo[] = {
	3.6, 3.6, 3.6,
	4.8, 12, 3.6, 4.8, 12, 3.6,
	2,
	3.6, 3.6, 3.6,
	4.8, 12, 3.6, 4.8, 12, 3.6
};

//Alert melody
static uint16_t alert_melody[] = {
    NOTE_G4, NOTE_B4, NOTE_D5, NOTE_G5, NOTE_B5,0,
    NOTE_GS4, NOTE_C4, NOTE_DS5, NOTE_GS5, NOTE_C5,0,
    NOTE_A5, NOTE_D4, NOTE_F5, NOTE_AS5, NOTE_D5
};

//Alert tempo
static float alert_tempo[] = {
    25, 25, 25, 25, 25,100,
    25, 25, 25, 25, 25,100,
    25, 25, 25, 25, 25
};

melody_t melody[NB_SONGS] = {
	//MARIO
	{
		.notes = mario_melody,
		.tempo = mario_tempo,
		.length = sizeof(mario_melody)/sizeof(uint16_t),

	},
	//UNDERWORLD
	{
		.notes = underworld_melody,
		.tempo = underworld_tempo,
		.length = sizeof(underworld_melody)/sizeof(uint16_t),
	},
	//STARWARS
	{
		.notes = starwars_melody,
		.tempo = starwars_tempo,
		.length = sizeof(starwars_melody)/sizeof(uint16_t),
	},
    //ALERT
    {
        .notes = alert_melody,
        .tempo = alert_tempo,
        .length = sizeof(alert_melody)/sizeof(uint16_t),
        
    },
};

void play_note(uint16_t note, uint16_t duration_ms) {
    dac_start();
	if(note != 0){
		dac_play(note);
	}
	chThdSleepMilliseconds(duration_ms);
	dac_stop();
}

static THD_WORKING_AREA(waPlayMelodyThd, 128);
static THD_FUNCTION(PlayMelodyThd, arg) {

    chRegSetThreadName("PlayMelody Thd");

	(void)arg;

	static melody_t* song = NULL;

    while(1){
		//this thread is wating until it receives a message
        chSysLock();
        song = (melody_t*) chThdSuspendS(&play_melody_ref);
        chSysUnlock();
        play=true;
        for (int thisNote = 0; thisNote < song->length; thisNote++) {

            if(!play){
                
                break;
            }

            uint16_t noteDuration = (uint16_t)(1000 / song->tempo[thisNote]);
            
            play_note(song->notes[thisNote], noteDuration);
            
            uint16_t pauseBetweenNotes = (uint16_t)(noteDuration * 1.30);
            
            chThdSleepMilliseconds(pauseBetweenNotes);
        }
        chBSemSignal(&endPlaying_sem);
        play=false;
    }
}


void play_melody_start(void){
	chThdCreateStatic(waPlayMelodyThd, sizeof(waPlayMelodyThd), NORMALPRIO, PlayMelodyThd, NULL);
}

void play_melody(song_selection_t choice){


	melody_t* song = &melody[choice];
    if(play==true){
        play=false;
        chBSemWait(&endPlaying_sem);
    }
	//if the refercence is NULL, then the thread is already running
	//when the refercence becomes not NULL, it means the thread is waiting
	if(play_melody_ref != NULL){
		//tell the thread to play the song given
		chThdResume(&play_melody_ref, (msg_t) song);
	}
}

void wait_until_melody_end(void){
    if(play == true){
        chBSemWait(&endPlaying_sem);
    }
}

void stop_current_melody(void){
    play = false;
    
}

