#ifndef ESP_ROS_H_
#define ESP_ROS_H_
#include "user_config.h" // It must be located above ros2arduino.h.
#include <Arduino.h>
#include "ESP32mother.h"
#include <ros2arduino.h>

#if defined __cplusplus
extern "C" {
#endif

Speed Tw_Vx(Speed_V,ROBOT_R*ROBOT_m);
Speed Tw_Vy(Speed_V,ROBOT_R*ROBOT_m);
Speed Tw_Angular(Speed_W,ROBOT_R*ROBOT_m);

void publish_odom(nav_msgs::Odometry* msg, void* arg)
{
  (void)(arg);
  nav_msgs::Odometry odom;
  msg->header.stamp = ros2::now();
  sprintf(msg->header.frame_id, "odom");
  msg->pose.pose.position.x = odm.x(ROBOT_m);
  msg->pose.pose.position.y = odm.y(ROBOT_m);
  msg->pose.pose.position.z = 0.0;
  float q0 = imu.calcQuat(imu.qw);
  float q1 = -1*imu.calcQuat(imu.qx);
  float q2 = -1*imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w=q0;
  odom_quat.x=q1;
  odom_quat.y=q2;
  odom_quat.z=q3;
  msg->pose.pose.orientation = odom_quat;
  //set the velocity
  sprintf(msg->child_frame_id, "base_link");
  msg->twist.twist.linear.x = odm.Rx(ROBOT_m);
  msg->twist.twist.linear.y = 0;
  msg->twist.twist.angular.z = odm.Angular(ROBOT_RAD);
}
void subscribe_twist(geometry_msgs::Twist* msg, void* arg)
{
  (void)(arg);
  Tw_Vx = msg->linear.x;
  Tw_Vy = msg->linear.y;
  Tw_Angular = msg->angular.z;
	ESP32M.update();
	float BTL=BatteryLevel();
	if(BTL<LowLevel&&BTL!=0.f)
		ESP32M.BTError();
	Vy = (float)Tw_Vy;
	Vx = (float)Tw_Vx;
	Angular = (float)Tw_Angular;
	if(((l1.swread(sw1)||l1.swread(sw2))&&Vx>0))
		wheel->Stop();
	else
		wheel->Move(Vx, Vy, Angular);
}

class F11RoboNode : public ros2::Node
{
public:
  F11RoboNode()
  : Node("F11Robo_node")
  {
    //ros2::Publisher<nav_msgs::Odometry>* odom_pub_ = this->createPublisher<nav_msgs::Odometry>("odom");
    //this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publish_odom, nullptr, odom_pub_);
    //this->createSubscriber<geometry_msgs::Twist>("cmd_vel", (ros2::CallbackFunc)subscribe_twist, nullptr);
  }
};

#if defined __cplusplus
}
#endif
#endif