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

// how to use
// ik_state_t * ik_state_ptr;
// ik_input_t ik_input;
// ...
// input target pos
// input target rpy with mkdh_rpy
// init_ik_lib(&ik_state_ptr, &ik_input);
// ...
// free_ik_lib(ik_state_ptr);
// mkdh rpy stands for making DH frame from the frame of target rpy

// public methods for Calculation at main()
void init_ik_lib(ik_state_t **ik_state_ptr_addr, ik_input_t *in_input);
void free_ik_lib(ik_state_t *ik_state_ptr);
void get_6joint_angles(ik_state_t *ik_state_ptr);

// private methods
radian_t get_q(int joint);
void set_q(int joint, radian_t new_q);
radian_t get_theta(int joint);
radian_t get_alpha(int joint);
meter_t get_a(int joint);
meter_t get_d(int joint);
meter_t get_hypothenuse(meter_t a, meter_t b);
radian_t get_angle_from_cosine_law(meter_t a, meter_t b, meter_t c);
void get_center_pos(v3_t *center_pos_ptr, type_t r0g[MAX_LEN][MAX_LEN], v3_t *pos_ptr);
void get_r0g(ik_state_t *ik_state_ptr);
void get_r36(ik_state_t *ik_state_ptr);
void get_q1q2q3(v3_t *center_pos_ptr);
void get_q4q5q6(type_t r36[MAX_LEN][MAX_LEN]);
void set_target_pos(ik_state_t *ik_state_ptr, meter3_t *target_pos_ptr);
void set_target_rpy(ik_state_t *ik_state_ptr, radian3_t *target_rpy_ptr);
void set_target_trigonometric_rpy(ik_state_t *ik_state_ptr);
void set_const_mkdh_rpy(ik_state_t *ik_state_ptr, radian3_t *mkdh_rpy_ptr);
void set_const_trigonometric_alpha(ik_state_t *ik_state_ptr);
void set_const_trigonometric_mkdh_rpy(ik_state_t *ik_state_ptr);
void set_var_trigonometric_theta(ik_state_t *ik_state_ptr);

// optional methods
void get_rpy_from_z60_x60_dhframe(radian3_t* dst_rpy, meter3_t* src_z60, meter3_t* src_x60);
void get_rotx(type_t dst[MAX_LEN][MAX_LEN], radian_t angle);
void get_roty(type_t dst[MAX_LEN][MAX_LEN], radian_t angle);
void get_rotz(type_t dst[MAX_LEN][MAX_LEN], radian_t angle);
void get_rotx_with_joint(type_t dst[MAX_LEN][MAX_LEN], int joint);
void get_roty_with_joint(type_t dst[MAX_LEN][MAX_LEN], int joint);
void get_rotz_with_joint(type_t dst[MAX_LEN][MAX_LEN], int joint);
void get_pose(type_t T[3+1][3+1], int idx);
void get_transpose_mat(type_t (*dst)[MAX_LEN], type_t (*src)[MAX_LEN], int row, int col);
void get_mat_mult(type_t (*dst)[MAX_LEN], type_t (*src0)[MAX_LEN], type_t (*src1)[MAX_LEN], v3_t * row_mid_col);
void mat_cpy(type_t (*dst)[6], type_t (*src)[6]);
void swap(type_t *a, type_t *b);
void get_eye_mat(type_t (*dst)[6]);
int get_inv_mat(type_t (*dst)[6], type_t (*src)[6]);

#endif