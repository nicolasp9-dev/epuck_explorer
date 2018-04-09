/*
 * File : mod_secure_conv.h
 * Project : e_puck_project
 * Description : Module in charge of data conversion
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#include "mod_secure_conv.h"
#include "mod_errors.h"
#define INT_MAX 65535


int size_t2int(size_t val) {
    if(val <= INT_MAX){
        return (int)((size_t)val);
    }
    error(CONV_OVERFLOW);
    return -1;
}

size_t int2size_t(int val) {
    if(val > 0){
        return (size_t)((unsigned)val);
    }
    error(CONV_OVERFLOW);
    return __SIZE_MAX__;
}
