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

#define IMAGE_BUFFER_SIZE        19200

static uint8_t * imagePtr;

void mod_img_init(void){
    if(dcmi_start()) error(DCMI_CAMERA_MEM_ALLOC);
    po8030_start();
    imagePtr = malloc(IMAGE_BUFFER_SIZE*sizeof(uint8_t));
    po8030_advanced_config(FORMAT_RGB565, 240, 160, 160, 160, SUBSAMPLING_X4, SUBSAMPLING_X4);
    dcmi_enable_double_buffering();
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    
}


void mod_image_capture(void){
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

    
}

void mod_image_sendPicture(int x, int y){
    mod_image_capture();
    char imageInfos[70];
    sprintf(imageInfos, "Image*%d,%d", x, y);
    mod_com_writeDatas(imageInfos, (char*) imagePtr,  IMAGE_BUFFER_SIZE);
}
