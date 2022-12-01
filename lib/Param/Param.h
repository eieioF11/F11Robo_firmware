#ifndef PARAM_H_
#define PARAM_H_
#include <stdio.h>
#include "math.h"
//---------------------User setup-------------------
#define ROBOT_R 33.35f //Wheel radius [mm]
#define ROBOT_L 105.0f //Distance from the center of the body to the wheel [mm]
//Mecanum Wheel only
#define ROBOT_A 5.f //Body length [mm]
#define ROBOT_B 5.f //Body length [mm]
/*
|<----A---->|
[/]-------[\] -
 |         |  |
 |  ROBOT  |  B
 |         |  |
[\]-------[/] -
*/
//--------------------------------------------------
/*Angle Unit*/
#define ROBOT_DEG 1
#define ROBOT_RAD DEG_TO_RAD
/*Distance Unit*/
#define ROBOT_mm 1.f
#define ROBOT_cm 0.1f
#define ROBOT_m 0.001f
#endif
