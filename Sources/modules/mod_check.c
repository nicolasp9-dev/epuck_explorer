/*
 * File : mod_check.h
 * Project : e_puck_project
 * Description : Module in charge of check for coherency in datas (asserts...)
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */

#include "headers/mod_check.h"
#include "headers/mod_errors.h"

void assert(_Bool value){
    if(value == 0){
        error(ASSERT_ERROR);
    }
    
}
