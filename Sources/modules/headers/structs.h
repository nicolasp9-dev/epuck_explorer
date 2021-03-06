/*
 * File : mod_structs.h
 * Project : e_puck_project
 * Description : Shared structs definition for multiples files access
 *
 * Written by Maxime Marchionno and Nicolas Peslerbe, April 2018
 * MICRO-315 | École Polytechnique Fédérale de Lausanne
 */


#ifndef _MOD_STRUCTS_
#define _MOD_STRUCTS_


typedef struct  {
    float left;
    float right;
}wheelSpeed_t;

typedef struct  {
    int mainSpeed;
    float angle;
}robotSpeed_t;


#endif
