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

typedef enum{
    CONTINUE=0,
    NEW
} exploration_t;

/**
 * @brief Discover the area to find borders, repport it into the mapping
 *
 * param[in] type   Precise if a new exploration needs to be done (or if it's just improvement of the last one)
 */
void mod_explo_discoverTheAreaOnThread(void);

/**
 * @brief   Explore the area to find objects, send image for processing
 *          and repport everything int the mapping
 */
void mod_explo_explorateTheAreaOnThread(exploration_t type);

/**
 * @brief   Move object depending of their classification
 */
void mod_explo_sortTheAreaOnThread(void);

void mod_explo_sendTheMap(void);

void mod_explo_waitUntilEndOfWork(void);

void mod_explo_init(void);
#endif
