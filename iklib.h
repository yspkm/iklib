/*
 * Copyright (c) 2022 Yosep Kim.
 * See LICENSE for more information
 * https://github.com/ypskm/iklib
 */

// one set of calculation consumes
// average 1.794872 clock cycles
// measured from 1000000 loop average 

#ifndef IKLIB_H
#define IKLIB_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "iklib_dh.h"

#ifndef PI
#define PI 3.14159265358979
#endif
#define DUMMY 0
#define ig 7

#define GET 0xA
#define SET 0xB
#define MAX_LEN 4

typedef double type_t;
typedef type_t meter_t;
typedef type_t radian_t;
typedef struct
{
    type_t v0;
    type_t v1;
    type_t v2;
} v3_t;
typedef struct
{
    type_t v0;
    type_t v1;
} v2_t;
typedef v3_t radian3_t;
typedef v3_t meter3_t;
typedef struct __ik_state_s
{
    type_t r36[MAX_LEN][MAX_LEN];
    type_t r0g[MAX_LEN][MAX_LEN];

    type_t cos_alpha[1 + 6 + 1];
    type_t sin_alpha[1 + 6 + 1];
    type_t cos_theta[1 + 6 + 1];
    type_t sin_theta[1 + 6 + 1];

    meter3_t target_pos;
    radian3_t target_rpy;
    radian3_t mkdh_rpy;

    type_t cos_yaw, sin_yaw;
    type_t cos_pitch, sin_pitch;
    type_t cos_roll, sin_roll;
    type_t cos_mkdh_yaw, sin_mkdh_yaw;
    type_t cos_mkdh_pitch, sin_mkdh_pitch;
    type_t cos_mkdh_roll, sin_mkdh_roll;
} ik_state_t;

typedef struct __ik_input_s
{
    meter3_t target_pos;
    radian3_t target_rpy;
    radian3_t mkdh_rpy;
} ik_input_t;

// public methods for Calculation at main()
void init_ik_lib(ik_state_t *ik_state_ptr, ik_input_t *in_input);
void free_ik_lib(ik_state_t *ik_state_ptr);
void get_6joint_angles(ik_state_t *ik_state_ptr);

// get joint angles
radian_t get_theta(int joint);

// optional
// get rpy from independent two basis (x, z) of frame 6/0
void get_rpy_from_z60_x60_dhframe(radian3_t* dst_rpy, meter3_t* src_z60, meter3_t* src_x60);
// get the values of free variables
radian_t get_q(int joint);

#endif