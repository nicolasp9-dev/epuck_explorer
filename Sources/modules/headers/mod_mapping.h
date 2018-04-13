/*
 * File : mod_mapping.h
 * Project : e_puck_project
 * Description : Module that compute and store datas of the domain
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_MAPPING_
#define _MOD_MAPPING_

/**
 * @brief Init needed libraries by mapping + Change position of the robot to zero
*/
void mod_mapping_init(void);

/**
 * @brief Calibrate sensors and motors to have correct accuracy
 */
void mod_mapping_calibrateTheSystem(void);


/**
 */
void mod_mapping_doInitialMapping(void);

#endif
