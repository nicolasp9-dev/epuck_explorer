/*
 * File : mod_image.c
 * Project : e_puck_project
 * Description : Module in charge of image acquisition and processing
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "headers/mod_errors.h"
#include "headers/mod_image.h"
#include "camera/dcmi_camera.h"
#include "camera/po8030.h"
#include "mod_communication.h"
#include "stdio.h"
#include "mod_check.h"
#include "mod_basicIO.h"
#include "mod_audio.h"

#define IMAGE_WALDY_BIAS 1000
#define IMAGE_BUFFER_SIZE        19200

static uint8_t * imagePtr;

void mod_image_happyToSeeWally(void){
    mod_audio_alertInterruption(SHORT);
    mod_audio_waitUntilMelodyEnd();
}


void mod_image_whereIsWally(void){
    int redcount = 0;
    int bluecount = 0;
    for(int i = 0 ; i < 80*120 ; i+=(80/2)){
        redcount += (int)((imagePtr[i]&0xF8)>>3);
        bluecount += (int)(imagePtr[i]&0x001F);
    }
    char toSend[100];
    sprintf(toSend, "redsum %d, bluesum%d",  redcount,  bluecount);
    mod_com_writeMessage(toSend, 3);
    
    if((redcount-bluecount)>IMAGE_WALDY_BIAS) mod_image_happyToSeeWally();
}

void mod_img_init(void){
    if(dcmi_start()) error(DCMI_CAMERA_MEM_ALLOC);
    po8030_start();
    imagePtr = malloc(IMAGE_BUFFER_SIZE*sizeof(uint8_t));
    po8030_advanced_config(FORMAT_RGB565, 160, 0, 320, 480, SUBSAMPLING_X4, SUBSAMPLING_X4);
    dcmi_disable_double_buffering();
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    
}


void mod_image_capture(void){
    mod_basicIO_changeRobotState(ALL_OFF);
    chThdSleepMilliseconds(400);
    if(dcmi_prepare()) error(DCMI_CAMERA_SIZE_NOT_FIT);
    //starts a capture
    dcmi_capture_start();
    //waits for the capture to be done
    wait_image_ready();
    ///understand
    
    assert(imagePtr);
    
    //gets the pointer to the array filled with the last image in RGB565
    imagePtr = dcmi_get_last_image_ptr();
    //Extracts only the red pixels
    chThdSleepMilliseconds(100);
    mod_basicIO_changeRobotState(WIP);

    
}

void mod_image_sendPicture(int x, int y){
    mod_image_capture();
    mod_image_whereIsWally();
    char imageInfos[70];
    sprintf(imageInfos, "Image:%d:%d: ", x, y);
    mod_com_writeDatas(imageInfos, (char*) imagePtr,  IMAGE_BUFFER_SIZE);
}


