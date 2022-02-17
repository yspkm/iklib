/*
 * Copyright (c) 2022 Yosep Kim.
 * See LICENSE for more information
 * https://github.com/ypskm/iklib
 */

#include "iklib.h"

// private functions
void set_q(int joint, radian_t new_q);
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

// it can be modified for different designs
void get_q1q2q3(v3_t *center_pos)
{
    meter_t x = center_pos->v0;
    meter_t y = center_pos->v1;
    meter_t z = center_pos->v2;
    meter_t ld4a3 = get_hypothenuse(get_d(4), -get_a(3));
    radian_t phi = atan2(get_d(4), -get_a(3));
    meter_t x_prime = get_hypothenuse(x, y);
    meter_t mx = x_prime - get_a(1);
    meter_t my = z - get_d(1);
    meter_t m = get_hypothenuse(mx, my);
    radian_t alpha = atan2(my, mx);
    radian_t gamma = get_angle_from_cosine_law(ld4a3, get_a(2), m);
    radian_t beta = get_angle_from_cosine_law(m, get_a(2), ld4a3);

    set_q(1, atan2(y, x));
    set_q(2, PI / 2 - beta - alpha);
    set_q(3, -(gamma - phi));
}

type_t get_set_q(int joint, int get_or_set, radian_t dummy_or_set)
{
    static radian_t q[7 + 1];
    if (get_or_set == GET)
    {
        return q[joint];
    }
    else
    {
        q[joint] = dummy_or_set;
        return 0;
    }
}

radian_t get_q(int joint)
{
    return get_set_q(joint, GET, 0);
}

void set_q(int joint, radian_t new_q)
{
    get_set_q(joint, SET, new_q);
}

radian_t get_theta(int joint)
{
    radian_t ret = 0.0;
    switch (joint)
    {
    case 1:
        ret = DH_THETA1(get_q(1));
        break;
    case 2:
        ret = DH_THETA2(get_q(2));
        break;
    case 3:
        ret = DH_THETA3(get_q(3));
        break;
    case 4:
        ret = DH_THETA4(get_q(4));
        break;
    case 5:
        ret = DH_THETA5(get_q(5));
        break;
    case 6:
        ret = DH_THETA6(get_q(6));
        break;
    case ig:
        ret = 0;
        break;
    }
    return ret;
}

radian_t get_alpha(int joint)
{
    radian_t ret = 0.0;
    switch (joint)
    {
    case 1:
        ret = DH_ALPHA1;
        break;
    case 2:
        ret = DH_ALPHA2;
        break;
    case 3:
        ret = DH_ALPHA3;
        break;
    case 4:
        ret = DH_ALPHA4;
        break;
    case 5:
        ret = DH_ALPHA5;
        break;
    case 6:
        ret = DH_ALPHA6;
        break;
    case ig:
        ret = 0;
        break;
    }
    return ret;
}

meter_t get_a(int joint)
{
    radian_t ret = 0.0;
    switch (joint)
    {
    case 1:
        ret = DH_A1;
        break;
    case 2:
        ret = DH_A2;
        break;
    case 3:
        ret = DH_A3;
        break;
    case 4:
        ret = DH_A4;
        break;
    case 5:
        ret = DH_A5;
        break;
    case 6:
        ret = DH_A6;
        break;
    case ig:
        ret = 0;
        break;
    }
    return ret;
}

meter_t get_d(int joint)
{
    radian_t ret = 0.0;
    switch (joint)
    {
    case 1:
        ret = DH_D1;
        break;
    case 2:
        ret = DH_D2;
        break;
    case 3:
        ret = DH_D3;
        break;
    case 4:
        ret = DH_D4;
        break;
    case 5:
        ret = DH_D5;
        break;
    case 6:
        ret = DH_D6;
        break;
    case ig:
        ret = DH_DG;
        break;
    }
    return ret;
}

meter_t get_hypothenuse(meter_t a, meter_t b)
{
    return sqrt(a * a + b * b);
}

radian_t get_angle_from_cosine_law(meter_t a, meter_t b, meter_t c)
{
    type_t cos_gamma = (a * a + b * b - c * c) / (2 * a * b);
    type_t sin_gamma = sqrt(1 - cos_gamma * cos_gamma);
    return atan2(sin_gamma, cos_gamma);
}

void get_center_pos(v3_t *center_pos, type_t r0g[MAX_LEN][MAX_LEN], v3_t *pos)
{
    type_t xg = pos->v0, yg = pos->v1, zg = pos->v2;
    type_t nx = r0g[0][2];
    type_t ny = r0g[1][2];
    type_t nz = r0g[2][2];
    type_t dg = get_d(ig);
    center_pos->v0 = xg - dg * nx;
    center_pos->v1 = yg - dg * ny;
    center_pos->v2 = zg - dg * nz;
}

void get_q4q5q6(type_t r36[MAX_LEN][MAX_LEN])
{
    set_q(4, atan2(r36[2][2], -r36[0][2]));
    set_q(5, atan2(sqrt(r36[0][2] * r36[0][2] + r36[2][2] * r36[2][2]), r36[1][2]));
    set_q(6, atan2(-r36[1][1], r36[1][0]));
}

void get_6joint_angles(ik_state_t *ik_state)
{
    meter3_t center_pos = {0};
    get_r0g(ik_state);
    get_center_pos(&center_pos, ik_state->r0g, &ik_state->target_pos);
    get_q1q2q3(&center_pos);

    set_var_trigonometric_theta(ik_state);
    get_r36(ik_state);
    get_q4q5q6(ik_state->r36);
}

void init_ik_lib(ik_state_t *ik_state_ptr, ik_input_t *in)
{
    set_target_pos(ik_state_ptr, &(in->target_pos));
    set_target_rpy(ik_state_ptr, &(in->target_rpy));
    set_const_mkdh_rpy(ik_state_ptr, &(in->mkdh_rpy));

    set_target_trigonometric_rpy(ik_state_ptr);
    set_const_trigonometric_mkdh_rpy(ik_state_ptr);
    set_const_trigonometric_alpha(ik_state_ptr);
}

void set_target_pos(ik_state_t *ik_state_ptr, meter3_t *target_pos_ptr)
{
    ik_state_ptr->target_pos.v0 = target_pos_ptr->v0;
    ik_state_ptr->target_pos.v1 = target_pos_ptr->v1;
    ik_state_ptr->target_pos.v2 = target_pos_ptr->v2;
}

void set_target_rpy(ik_state_t *ik_state_ptr, radian3_t *target_rpy_ptr)
{
    ik_state_ptr->target_rpy.v0 = target_rpy_ptr->v0;
    ik_state_ptr->target_rpy.v1 = target_rpy_ptr->v1;
    ik_state_ptr->target_rpy.v2 = target_rpy_ptr->v2;
}

void set_const_mkdh_rpy(ik_state_t *ik_state_ptr, radian3_t *mkdh_rpy)
{
    ik_state_ptr->mkdh_rpy.v0 = mkdh_rpy->v0;
    ik_state_ptr->mkdh_rpy.v1 = mkdh_rpy->v1;
    ik_state_ptr->mkdh_rpy.v2 = mkdh_rpy->v2;
}

void set_target_trigonometric_rpy(ik_state_t *ik_state_ptr)
{
    ik_state_ptr->cos_roll = cos(ik_state_ptr->target_rpy.v0);
    ik_state_ptr->sin_roll = sin(ik_state_ptr->target_rpy.v0);
    ik_state_ptr->cos_pitch = cos(ik_state_ptr->target_rpy.v1);
    ik_state_ptr->sin_pitch = sin(ik_state_ptr->target_rpy.v1);
    ik_state_ptr->cos_yaw = cos(ik_state_ptr->target_rpy.v2);
    ik_state_ptr->sin_yaw = sin(ik_state_ptr->target_rpy.v2);
}

void set_const_trigonometric_alpha(ik_state_t *ik_state_ptr)
{
    for (int i = 1; i <= 6; i++)
    {
        ik_state_ptr->cos_alpha[i] = cos(get_alpha(i));
        ik_state_ptr->sin_alpha[i] = sin(get_alpha(i));
    }
}
void set_const_trigonometric_mkdh_rpy(ik_state_t *ik_state_ptr)
{
    ik_state_ptr->cos_mkdh_roll = cos(ik_state_ptr->mkdh_rpy.v0);
    ik_state_ptr->sin_mkdh_roll = sin(ik_state_ptr->mkdh_rpy.v0);
    ik_state_ptr->cos_mkdh_pitch = cos(ik_state_ptr->mkdh_rpy.v1);
    ik_state_ptr->sin_mkdh_pitch = sin(ik_state_ptr->mkdh_rpy.v1);
    ik_state_ptr->cos_mkdh_yaw = cos(ik_state_ptr->mkdh_rpy.v2);
    ik_state_ptr->sin_mkdh_yaw = sin(ik_state_ptr->mkdh_rpy.v2);
}

void set_var_trigonometric_theta(ik_state_t *ik_state_ptr)
{
    for (int i = 1; i <= 6; i++)
    {
        ik_state_ptr->cos_theta[i] = cos(get_theta(i));
        ik_state_ptr->sin_theta[i] = sin(get_theta(i));
    }
}

void get_r0g_00(ik_state_t *ik_state)
{
    ik_state->r0g[0][0] = ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_yaw - ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_yaw * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) - ik_state->sin_mkdh_pitch * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch);
}
void get_r0g_01(ik_state_t *ik_state)
{
    ik_state->r0g[0][1] = ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_roll * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch) - ik_state->cos_pitch * ik_state->cos_yaw * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) - (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll);
}
void get_r0g_02(ik_state_t *ik_state)
{
    ik_state->r0g[0][2] = (ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) + ik_state->cos_pitch * ik_state->cos_yaw * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch);
}
void get_r0g_10(ik_state_t *ik_state)
{
    ik_state->r0g[1][0] = ik_state->sin_mkdh_pitch * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_yaw * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->sin_yaw;
}
void get_r0g_11(ik_state_t *ik_state)
{
    ik_state->r0g[1][1] = (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) - ik_state->cos_pitch * ik_state->sin_yaw * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) - ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_roll * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw);
}
void get_r0g_12(ik_state_t *ik_state)
{
    ik_state->r0g[1][2] = ik_state->cos_pitch * ik_state->sin_yaw * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) - (ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) - ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw);
}
void get_r0g_20(ik_state_t *ik_state)
{
    ik_state->r0g[2][0] = ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->sin_mkdh_yaw * ik_state->sin_roll - ik_state->cos_pitch * ik_state->cos_roll * ik_state->sin_mkdh_pitch - ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->sin_pitch;
}
void get_r0g_21(ik_state_t *ik_state)
{
    ik_state->r0g[2][1] = ik_state->sin_pitch * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_pitch * ik_state->sin_roll * (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_roll * ik_state->sin_mkdh_roll;
}
void get_r0g_22(ik_state_t *ik_state)
{
    ik_state->r0g[2][2] = ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_roll - ik_state->cos_pitch * ik_state->sin_roll * (ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) - ik_state->sin_pitch * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch);
}

void get_r36_00(ik_state_t *ik_state)
{
    ik_state->r36[0][0] = -(ik_state->cos_alpha[2] * ik_state->sin_theta[3] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) - ik_state->cos_theta[3] * (ik_state->cos_theta[2] * ik_state->sin_theta[1] + ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->sin_theta[2]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[1] * ik_state->sin_theta[3]) * (ik_state->sin_mkdh_pitch * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_yaw * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->sin_yaw) - (ik_state->cos_theta[3] * (ik_state->cos_theta[1] * ik_state->cos_theta[2] - ik_state->cos_alpha[1] * ik_state->sin_theta[1] * ik_state->sin_theta[2]) - ik_state->cos_alpha[2] * ik_state->sin_theta[3] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->sin_theta[1] * ik_state->sin_theta[3]) * (ik_state->sin_mkdh_pitch * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_yaw * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) - ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_yaw) - (ik_state->cos_alpha[1] * ik_state->sin_alpha[2] * ik_state->sin_theta[3] + ik_state->sin_alpha[1] * ik_state->cos_theta[3] * ik_state->sin_theta[2] + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[3]) * (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->sin_pitch + ik_state->cos_pitch * ik_state->cos_roll * ik_state->sin_mkdh_pitch - ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->sin_mkdh_yaw * ik_state->sin_roll);
}
void get_r36_01(ik_state_t *ik_state)
{
    ik_state->r36[0][1] = (ik_state->cos_alpha[2] * ik_state->sin_theta[3] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) - ik_state->cos_theta[3] * (ik_state->cos_theta[2] * ik_state->sin_theta[1] + ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->sin_theta[2]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[1] * ik_state->sin_theta[3]) * (ik_state->cos_pitch * ik_state->sin_yaw * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) - (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_roll * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw)) - (ik_state->cos_theta[3] * (ik_state->cos_theta[1] * ik_state->cos_theta[2] - ik_state->cos_alpha[1] * ik_state->sin_theta[1] * ik_state->sin_theta[2]) - ik_state->cos_alpha[2] * ik_state->sin_theta[3] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->sin_theta[1] * ik_state->sin_theta[3]) * ((ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) + ik_state->cos_pitch * ik_state->cos_yaw * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) - ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_roll * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch)) + (ik_state->sin_pitch * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_pitch * ik_state->sin_roll * (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_roll * ik_state->sin_mkdh_roll) * (ik_state->cos_alpha[1] * ik_state->sin_alpha[2] * ik_state->sin_theta[3] + ik_state->sin_alpha[1] * ik_state->cos_theta[3] * ik_state->sin_theta[2] + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[3]);
}
void get_r36_02(ik_state_t *ik_state)
{
    ik_state->r36[0][2] = (ik_state->cos_theta[3] * (ik_state->cos_theta[1] * ik_state->cos_theta[2] - ik_state->cos_alpha[1] * ik_state->sin_theta[1] * ik_state->sin_theta[2]) - ik_state->cos_alpha[2] * ik_state->sin_theta[3] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->sin_theta[1] * ik_state->sin_theta[3]) * ((ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) + ik_state->cos_pitch * ik_state->cos_yaw * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch)) + (ik_state->cos_alpha[2] * ik_state->sin_theta[3] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) - ik_state->cos_theta[3] * (ik_state->cos_theta[2] * ik_state->sin_theta[1] + ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->sin_theta[2]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[1] * ik_state->sin_theta[3]) * ((ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) - ik_state->cos_pitch * ik_state->sin_yaw * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw)) - (ik_state->sin_pitch * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_pitch * ik_state->sin_roll * (ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) - ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_roll) * (ik_state->cos_alpha[1] * ik_state->sin_alpha[2] * ik_state->sin_theta[3] + ik_state->sin_alpha[1] * ik_state->cos_theta[3] * ik_state->sin_theta[2] + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[3]);
}
void get_r36_10(ik_state_t *ik_state)
{
    ik_state->r36[1][0] = (ik_state->sin_theta[3] * (ik_state->cos_theta[1] * ik_state->cos_theta[2] - ik_state->cos_alpha[1] * ik_state->sin_theta[1] * ik_state->sin_theta[2]) + ik_state->cos_alpha[2] * ik_state->cos_theta[3] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) - ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[3] * ik_state->sin_theta[1]) * (ik_state->sin_mkdh_pitch * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_yaw * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) - ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_yaw) - (ik_state->sin_theta[3] * (ik_state->cos_theta[2] * ik_state->sin_theta[1] + ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->sin_theta[2]) + ik_state->cos_alpha[2] * ik_state->cos_theta[3] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[1] * ik_state->cos_theta[3]) * (ik_state->sin_mkdh_pitch * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_yaw * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->sin_yaw) - (ik_state->cos_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[3] - ik_state->sin_alpha[1] * ik_state->sin_theta[2] * ik_state->sin_theta[3] + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[2] * ik_state->cos_theta[3]) * (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->sin_pitch + ik_state->cos_pitch * ik_state->cos_roll * ik_state->sin_mkdh_pitch - ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->sin_mkdh_yaw * ik_state->sin_roll);
}
void get_r36_11(ik_state_t *ik_state)
{
    ik_state->r36[1][1] = (ik_state->sin_theta[3] * (ik_state->cos_theta[2] * ik_state->sin_theta[1] + ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->sin_theta[2]) + ik_state->cos_alpha[2] * ik_state->cos_theta[3] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[1] * ik_state->cos_theta[3]) * (ik_state->cos_pitch * ik_state->sin_yaw * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) - (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_roll * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw)) + (ik_state->sin_theta[3] * (ik_state->cos_theta[1] * ik_state->cos_theta[2] - ik_state->cos_alpha[1] * ik_state->sin_theta[1] * ik_state->sin_theta[2]) + ik_state->cos_alpha[2] * ik_state->cos_theta[3] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) - ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[3] * ik_state->sin_theta[1]) * ((ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) + ik_state->cos_pitch * ik_state->cos_yaw * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) - ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_roll * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch)) + (ik_state->sin_pitch * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_pitch * ik_state->sin_roll * (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_roll * ik_state->sin_mkdh_roll) * (ik_state->cos_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[3] - ik_state->sin_alpha[1] * ik_state->sin_theta[2] * ik_state->sin_theta[3] + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[2] * ik_state->cos_theta[3]);
}
void get_r36_12(ik_state_t *ik_state)
{
    ik_state->r36[1][2] = (ik_state->sin_theta[3] * (ik_state->cos_theta[2] * ik_state->sin_theta[1] + ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->sin_theta[2]) + ik_state->cos_alpha[2] * ik_state->cos_theta[3] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) + ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[1] * ik_state->cos_theta[3]) * ((ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) - ik_state->cos_pitch * ik_state->sin_yaw * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw)) - (ik_state->sin_theta[3] * (ik_state->cos_theta[1] * ik_state->cos_theta[2] - ik_state->cos_alpha[1] * ik_state->sin_theta[1] * ik_state->sin_theta[2]) + ik_state->cos_alpha[2] * ik_state->cos_theta[3] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) - ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[3] * ik_state->sin_theta[1]) * ((ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) + ik_state->cos_pitch * ik_state->cos_yaw * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch)) - (ik_state->sin_pitch * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_pitch * ik_state->sin_roll * (ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) - ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_roll) * (ik_state->cos_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[3] - ik_state->sin_alpha[1] * ik_state->sin_theta[2] * ik_state->sin_theta[3] + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[2] * ik_state->cos_theta[3]);
}
void get_r36_20(ik_state_t *ik_state)
{
    ik_state->r36[2][0] = (ik_state->sin_alpha[2] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) - ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[1]) * (ik_state->sin_mkdh_pitch * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_yaw * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->sin_yaw) - (ik_state->sin_alpha[2] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->sin_theta[1]) * (ik_state->sin_mkdh_pitch * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_yaw * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) - ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_yaw) - (ik_state->cos_alpha[1] * ik_state->cos_alpha[2] - ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[2]) * (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_pitch * ik_state->sin_pitch + ik_state->cos_pitch * ik_state->cos_roll * ik_state->sin_mkdh_pitch - ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->sin_mkdh_yaw * ik_state->sin_roll);
}
void get_r36_21(ik_state_t *ik_state)
{
    ik_state->r36[2][1] = (ik_state->cos_alpha[1] * ik_state->cos_alpha[2] - ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[2]) * (ik_state->sin_pitch * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_pitch * ik_state->sin_roll * (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_roll * ik_state->sin_mkdh_roll) - (ik_state->sin_alpha[2] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->sin_theta[1]) * ((ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) + ik_state->cos_pitch * ik_state->cos_yaw * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) - ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_roll * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch)) - (ik_state->sin_alpha[2] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) - ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[1]) * (ik_state->cos_pitch * ik_state->sin_yaw * (ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw - ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) - (ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll + ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) + ik_state->cos_mkdh_pitch * ik_state->sin_mkdh_roll * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw));
}
void get_r36_22(ik_state_t *ik_state)
{
    ik_state->r36[2][2] = (ik_state->sin_alpha[2] * (ik_state->cos_theta[1] * ik_state->sin_theta[2] + ik_state->cos_alpha[1] * ik_state->cos_theta[2] * ik_state->sin_theta[1]) + ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->sin_theta[1]) * ((ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->sin_yaw - ik_state->cos_yaw * ik_state->sin_pitch * ik_state->sin_roll) + ik_state->cos_pitch * ik_state->cos_yaw * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * (ik_state->sin_roll * ik_state->sin_yaw + ik_state->cos_roll * ik_state->cos_yaw * ik_state->sin_pitch)) - (ik_state->cos_alpha[1] * ik_state->cos_alpha[2] - ik_state->sin_alpha[1] * ik_state->sin_alpha[2] * ik_state->cos_theta[2]) * (ik_state->sin_pitch * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_pitch * ik_state->sin_roll * (ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) - ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * ik_state->cos_pitch * ik_state->cos_roll) - (ik_state->sin_alpha[2] * (ik_state->sin_theta[1] * ik_state->sin_theta[2] - ik_state->cos_alpha[1] * ik_state->cos_theta[1] * ik_state->cos_theta[2]) - ik_state->cos_alpha[2] * ik_state->sin_alpha[1] * ik_state->cos_theta[1]) * ((ik_state->cos_mkdh_yaw * ik_state->sin_mkdh_roll - ik_state->cos_mkdh_roll * ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_pitch) * (ik_state->cos_roll * ik_state->cos_yaw + ik_state->sin_pitch * ik_state->sin_roll * ik_state->sin_yaw) - ik_state->cos_pitch * ik_state->sin_yaw * (ik_state->sin_mkdh_yaw * ik_state->sin_mkdh_roll + ik_state->cos_mkdh_yaw * ik_state->cos_mkdh_roll * ik_state->sin_mkdh_pitch) + ik_state->cos_mkdh_roll * ik_state->cos_mkdh_pitch * (ik_state->cos_yaw * ik_state->sin_roll - ik_state->cos_roll * ik_state->sin_pitch * ik_state->sin_yaw));
}

void get_r0g(ik_state_t *ik_state_ptr)
{
    get_r0g_00(ik_state_ptr);
    get_r0g_01(ik_state_ptr);
    get_r0g_02(ik_state_ptr);
    get_r0g_10(ik_state_ptr);
    get_r0g_11(ik_state_ptr);
    get_r0g_12(ik_state_ptr);
    get_r0g_20(ik_state_ptr);
    get_r0g_21(ik_state_ptr);
    get_r0g_22(ik_state_ptr);
}

void get_r36(ik_state_t *ik_state_ptr)
{
    get_r36_00(ik_state_ptr);
    get_r36_01(ik_state_ptr);
    get_r36_02(ik_state_ptr);
    get_r36_10(ik_state_ptr);
    get_r36_11(ik_state_ptr);
    get_r36_12(ik_state_ptr);
    get_r36_20(ik_state_ptr);
    get_r36_21(ik_state_ptr);
    get_r36_22(ik_state_ptr);
}

// Optional Methods
void get_rpy_from_z60_x60_dhframe(radian3_t *dst_rpy, meter3_t *src_z60, meter3_t *src_x60)
{
    // rpy: [roll; pitch; yaw]
    // x60: x6 axis (orthonormal basis) wrt frame 0
    // z60: z6 axis (orthonormal basis) wrt frame 0
    // z60->v0 == sin(pitch)
    // z60->v1 == -cos(pitch)*sin(roll)
    // z60->v2 == cos(pitch)*cos(roll)
    // x60->v0 == cos(pitch)*cos(yaw)
    dst_rpy->v1 = asin(src_z60->v0);
    dst_rpy->v0 = asin(src_z60->v1 / (-cos(dst_rpy->v1)));
    dst_rpy->v2 = acos(src_x60->v0 / cos(dst_rpy->v1));
}