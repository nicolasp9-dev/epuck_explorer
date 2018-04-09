/*
 * File : mod_image.h
 * Project : e_puck_project
 * Description : Module in charge of image acquisition and processing
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_IMAGE_
#define _MOD_IMAGE_

/**
 * @brief Initialize the video device
 */
void mod_img_init(void);

/**
 * @brief 
 */
void mod_image_capture(void);
#endif

