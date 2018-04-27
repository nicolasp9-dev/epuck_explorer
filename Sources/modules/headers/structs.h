/*
 * File : mod_structs.h
 * Project : e_puck_project
 * Description : Module in charge of check for coherency in datas (asserts...)
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_STRUCTS_
#define _MOD_STRUCTS_


typedef struct  {
    double left;
    double right;
}wheelSpeed_t;

typedef struct  {
    int mainSpeed;
    double angle;
}robotSpeed_t;


#endif
