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

void mod_img_init(void){
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    if(dcmi_start()) error(DCMI_CAMERA_MEM_ALLOC);
    if(dcmi_prepare()) error(DCMI_CAMERA_SIZE_NOT_FIT);
    
}

void mod_image_capture(void){
    
    
}
