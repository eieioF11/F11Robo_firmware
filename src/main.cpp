
#include "user_config.h" // It must be located above ros2arduino.h.
#include <Arduino.h>
#include "ESP32mother.h"
#include <ros2arduino.h>
#include "F11Robo_node.hpp"

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
			I2C_Error = (i2c_err_t)l1.com();
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
	ESP32M.setup();
	//ROS2設定
	while (!ESP_SERIAL);
	ros2::init(&ESP_SERIAL);
}
void publishString(std_msgs::String* msg, void* arg)
{
  (void)(arg);

  static int cnt = 0;
  sprintf(msg->data, "Hello ros2arduino %d", cnt++);
}

class StringPub : public ros2::Node
{
public:
  StringPub()
  : Node("ros2arduino_pub_node")
  {
    ros2::Publisher<std_msgs::String>* publisher_ = this->createPublisher<std_msgs::String>("arduino_chatter");
    this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishString, nullptr, publisher_);
  }
};

void loop()
{
  static StringPub StringNode;
  ros2::spin(&StringNode);
  //static F11RoboNode F11Robo_Node;
  //ros2::spin(&F11Robo_Node);
}