/*
 * Copyright (c) 2022 Yosep Kim.
 * See LICENSE for more information
 * https://github.com/ypskm/iklib
 */

// You can set different DH parameters
// by changing the literals below
#ifndef IKLIB_DH_H
#define IKLIB_DH_H

#ifndef PI
#define PI 3.14159265358979
#endif

// "Link Length"
// displacement between zi and zi+1
// along the direction of xi basis
#define DH_A1 0.35
#define DH_A2 1.25
#define DH_A3 -0.054
#define DH_A4 0
#define DH_A5 0
#define DH_A6 0

// "Twist Angle"
// angular displacement between zi and zi+1
// about the right-hand direction of xi basis
#define DH_ALPHA1 -PI / 2
#define DH_ALPHA2 0
#define DH_ALPHA3 -PI / 2
#define DH_ALPHA4 PI / 2
#define DH_ALPHA5 -PI / 2
#define DH_ALPHA6 0

// "Link Offset"
// displacement between xi-1 and xi
// along the direction of zi basis
#define DH_D1 0.75
#define DH_D2 0
#define DH_D3 0
#define DH_D4 1.5
#define DH_D5 0
#define DH_D6 0
#define DH_DG 0.303

// "Joint Angle"
// angular displacement between xi-1 and xi
// about the right-hand direction of zi basis
// since it can be linear function,
// It can be any Function of free variable.
#define DH_THETA1(Q) ((Q))
#define DH_THETA2(Q) ((Q)-PI / 2)
#define DH_THETA3(Q) ((Q))
#define DH_THETA4(Q) ((Q))
#define DH_THETA5(Q) ((Q))
#define DH_THETA6(Q) ((Q))

#endif