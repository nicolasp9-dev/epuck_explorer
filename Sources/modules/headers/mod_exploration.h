/*
 * File : mod_exploration.h
 * Project : e_puck_project
 * Description : Module in charge of exploration / autonomous displacements algorithms
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */



#ifndef _MOD_EXPLORATION_
#define _MOD_EXPLORATION_

#include <stdbool.h>

typedef struct{
    bool discovering;
    bool exploration;
} history_t;


extern history_t history;

/**
 * @brief Initialize the exploration module (and sub-modules)
 */
void mod_explo_initModule(void);


/**
 * @brief Discover the area to find borders, repport it into the mapping
 */
void mod_explo_discoverTheAreaOnThread(void);

/**
 * @brief   Explore the area to find objects, send image for processing
 *          and repport everything int the mapping
*/
void mod_explo_explorateTheAreaOnThread(void);

/**
 * @brief   Ask for the distant device to send the map to the user
 */
void mod_explo_sendTheMap(void);

/**
 * @brief   Unlock when work is finished
 */
void mod_explo_waitUntilEndOfWork(void);

void mod_explo_doDefinedPath(void);

#endif
