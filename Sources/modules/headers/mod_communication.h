/*
 * File : mod_communication.h
 * Project : e_puck_project
 * Description : Module in charge of bluetooth communcations with distant device
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_COMMUNICATION_
#define _MOD_COMMUNICATION_


#include <stdlib.h>

/**
 * @brief Initialize the serial connexion to be able to send datas
 */
void mod_com_initConnexion(void);

/**
 * @brief Write some datas on the serial port
 * @details It introduce a header with datas size & communication title
 *
 * @param[in] type          The title of the content that will be send (image...)
 * @param[in] toWrite       A pointer to the datas that need to be send
 * @param[in] toWriteSize   The size of the data to send. If the datas have a end
 *                          character it can be 0.
 */
void mod_com_writeDatas(char* type, char* toWrite, size_t toWriteSize);


#endif
