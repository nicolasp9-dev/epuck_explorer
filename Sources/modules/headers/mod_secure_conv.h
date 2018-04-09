/*
 * File : mod_secure_conv.h
 * Project : e_puck_project
 * Description : Module in charge of data conversion
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_SECURE_CONV_
#define _MOD_SECURE_CONV_

#include <stdlib.h>


// Adapted from https://stackoverflow.com/questions/27490762/how-can-i-convert-to-size-t-from-int-safely
int size_t2int(size_t val);
size_t int2size_t(int val);



#endif
