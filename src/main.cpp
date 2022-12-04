#include <Arduino.h>
#include "ESP32mother.h"
//#include "F11Robo_node.hpp"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>

rclc_executor_t executor;
rcl_publisher_t pub_odom;
rcl_subscription_t sub_twist;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

geometry_msgs__msg__Twist recv_twist_msg;

void publish_odom(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		nav_msgs__msg__Odometry odom;
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		odom.header.stamp.sec = ts.tv_sec;
		odom.header.stamp.nanosec = ts.tv_nsec;
		//odom.header.frame_id="odom";
		sprintf(odom.header.frame_id.data, "odom");
		odom.pose.pose.position.x = odm.x(ROBOT_m);
		odom.pose.pose.position.y = odm.y(ROBOT_m);
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation.w = imu.calcQuat(imu.qw);
		odom.pose.pose.orientation.x = -1*imu.calcQuat(imu.qx);
		odom.pose.pose.orientation.y = -1*imu.calcQuat(imu.qy);
		odom.pose.pose.orientation.z = imu.calcQuat(imu.qz);
		//set the velocity
		//odom.child_frame_id="base_link";
		sprintf(odom.child_frame_id.data, "base_link");
		odom.twist.twist.linear.x = odm.Rx(ROBOT_m);
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = odm.Angular(ROBOT_RAD);
		rcl_publish(&pub_odom, &odom, NULL);
	}
}

Speed Tw_Vx(Speed_V,ROBOT_R*ROBOT_m);
Speed Tw_Vy(Speed_V,ROBOT_R*ROBOT_m);
Speed Tw_Angular(Speed_W,ROBOT_R*ROBOT_m);
// Callback
void callback_twist(const void *raw_msg)
{
	geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist *)raw_msg;
	Tw_Vx = (float)msg->linear.x;
	Tw_Vy = (float)msg->linear.y;
	Tw_Angular = (float)msg->angular.z;
}

void motor_update()
{
	float BTL=BatteryLevel();
	if(BTL<LowLevel&&BTL!=0.f)
		ESP32M.BTError();
	Vy = (float)Tw_Vy;
	Vx = (float)Tw_Vx;
	Angular = (float)Tw_Angular;
	if(((l1.swread(sw1)||l1.swread(sw2))&&Vx>0)||ESP32M.EMARGENCYSTOP())
		wheel->Stop();
	else
		wheel->Move(Vx, Vy, Angular);
}


void setup()
{
	ESPtask_i2c_init = []()
	{
		if (imu.begin() != INV_SUCCESS)
		{
			while (1)
			{
				Serial.println("Unable to communicate with MPU-9250");
				Serial.println("Check connections, and try again.");
				Serial.println();
				delay(5000);
			}
		}
		imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |  // Enable 6-axis quat
						 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
					 10);					   // Set DMP FIFO rate to 10 Hz
											   //mpu.setup();
	}; //i2cを使用するものの初期化
	ESPtask_i2c_while = []() {
		static int ct = 0;
		switch (ct)
		{
		case 0:
			l1.com();
		case 1:
			// Check for new data in the FIFO
			if (imu.fifoAvailable())
			{
				// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
				if (imu.dmpUpdateFifo() == INV_SUCCESS)
				{
					// computeEulerAngles can be used -- after updating the
					// quaternion values -- to estimate roll, pitch, and yaw
					imu.computeEulerAngles();
				}
			}
			ct = -1;
			break;
		}
		ct++;
	};		   //i2cを使用するものをsensor_interval[ms]ごとに実行
	Serial.begin(115200);
	ESP32M.setup();
	digitalWrite(STBY, HIGH);
	set_microros_transports();

	delay(2000);

	allocator = rcl_get_default_allocator();

	// create init_options
	rclc_support_init(&support, 0, NULL, &allocator);
	rclc_node_init_default(&node, "micro_ros_node", "", &support);
	rclc_publisher_init(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom",&rmw_qos_profile_default);
	//const unsigned int timer_timeout = 1000;
	//rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), publish_odom);

	// Subscriber 作成
	rclc_subscription_init(&sub_twist, &node,
							ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
							"cmd_vel",&rmw_qos_profile_default
	);

	const int callback_size = 2;	// コールバックを行う数
	rclc_executor_init(&executor, &support.context, callback_size, &allocator);
	//rclc_executor_add_timer(&executor, &timer);
	rclc_executor_add_subscription(&executor, &sub_twist, &recv_twist_msg, 
								   &callback_twist, ON_NEW_DATA);
	digitalWrite(StatusLED, LOW);
	digitalWrite(STBY, LOW);
}

void loop()
{
	delay(1);
	ESP32M.update();
	motor_update();
		nav_msgs__msg__Odometry odom;
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		odom.header.stamp.sec = ts.tv_sec;
		odom.header.stamp.nanosec = ts.tv_nsec;
		//odom.header.frame_id="odom";
		sprintf(odom.header.frame_id.data, "odom");
		odom.pose.pose.position.x = odm.x(ROBOT_m);
		odom.pose.pose.position.y = odm.y(ROBOT_m);
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation.w = imu.calcQuat(imu.qw);
		odom.pose.pose.orientation.x = -1*imu.calcQuat(imu.qx);
		odom.pose.pose.orientation.y = -1*imu.calcQuat(imu.qy);
		odom.pose.pose.orientation.z = imu.calcQuat(imu.qz);
		//set the velocity
		//odom.child_frame_id="base_link";
		sprintf(odom.child_frame_id.data, "base_link");
		odom.twist.twist.linear.x = odm.Rx(ROBOT_m);
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = odm.Angular(ROBOT_RAD);
		rcl_publish(&pub_odom, &odom, NULL);
	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
}