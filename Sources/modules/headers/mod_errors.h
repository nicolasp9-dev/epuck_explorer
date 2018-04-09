/*
 * File : errors.h
 * Project : e_puck_project
 * Description : Module in charge of giving errors feedback
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_ERRORS_
#define _MOD_ERRORS_

/**
 * @brief   List of possible errors, usefull to identify which error is happening
 */
typedef enum {
    ASSERT_ERROR=1, // Error 1 - General
    CONV_OVERFLOW, // Error 2 - mod_secure_conv.c
    DCMI_CAMERA_MEM_ALLOC, // Error 3 - mod_image.c
    DCMI_CAMERA_SIZE_NOT_FIT, // Error 4 - mod_image.c
    NO_COMMAND, // Error 5 - main.c
} error_t;

/**
 * @brief   Function to inform the user of a critical error with an error code
 *          that correspond to the number of front led blink.
 */
void error(error_t type);

#endif
