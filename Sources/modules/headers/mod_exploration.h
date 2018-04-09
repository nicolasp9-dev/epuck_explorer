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

/**
 * @brief Discover the area to find borders, repport it into the mapping
 */
void mod_explo_discoverArea(void);

/**
 * @brief   Explore the area to find objects, send image for processing
 *          and repport everything int the mapping
 */
void mod_explo_exploreArea(void);

/**
 * @brief   Move object depending of their classification
 */
void mod_explo_sortArea(void);

#endif
