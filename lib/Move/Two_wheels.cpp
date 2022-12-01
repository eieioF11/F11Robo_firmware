#include "Two_wheels.h"

Two_wheels::Two_wheels(MD **md_):MoveBase(md_){}

void Two_wheels::Move(float V_x,float V_y,float V_angle)
{
  md[0]->Move(V_x/(ROBOT_R*ROBOT_m)-(V_angle*ROBOT_L*ROBOT_m)/(ROBOT_R*ROBOT_m));
  md[1]->Move(V_x/(ROBOT_R*ROBOT_m)+(V_angle*ROBOT_L*ROBOT_m)/(ROBOT_R*ROBOT_m));
}

void Two_wheels::Stop()
{
  md[0]->Brake();
  md[1]->Brake();
}