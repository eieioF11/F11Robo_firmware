#ifndef ODOMETRY_H_
#define ODOMETRY_H_
#include <Arduino.h>
#include <stdio.h>
#include "math.h"
#include "Kalman.h"
#include "lowpassfilter.h"
#include "Param.h"

#if defined __cplusplus
extern "C"
{
#endif

//Gyro sensor Z-axis error
#define GyroZrange 0.2f

typedef enum
{
	NONE = 0,
	TWOWHEEL,
	OMUNI2,
	OMUNI3,
	OMUNI4,
	MECANUM4
} odmmode;

class Odometry
{
	private:
		//Filter
		Kalman kalmanX;
		Kalman kalmanY;
		Kalman kalmanZ;
		Kalman kalmanYaw;
		LowpassFilter ARoll;
		LowpassFilter APitch;
		//LowpassFilter AYaw;
		float AYaw;
		//Sensor  value
		float ax, ay, az;
		float gx, gy, gz;
		float mx, my, mz;
		float w[4]; //[rad/s]
		float W,angular;
		//coordinates
		float X;
		float Y;
		float wYaw;		  //wheel yaw
		float rx, ry, rc; //Robot coordinates rx[mm],ry[mm],rc[rad]
		//Posture
		float Pitch;
		float Roll;
		float Yaw;
		float ini_Pitch;
		float ini_Roll;
		float ini_Yaw;
		//Posture angle from accelerometer and magnetic sensor
		float Aroll;
		float Apitch;
		float Ayaw;
		//Time
		float dt;
		unsigned long nowtime, oldtime;
		//mode
		odmmode mode;
		uint8_t sensor;
		//Function
		void Accelposture();
		void Magposture();
		void update_posture(float dt);
		float update_time();
	public:
		Odometry();
		//Setup
		void setup(odmmode);
		bool zeroset(int n=10);//Zero point setting
		//Set value
		void setposture(float Pitch, float Roll, float Yaw);														//[deg] Execute with update () function
		void setnineaxis(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz); //ax,ay,az[G] gx,gy,gz[deg/s] mx,my,mz[mGauss] Execute with update () function
		void setsixaxis(float ax, float ay, float az, float gx, float gy, float gz);								//ax,ay,az[G] gx,gy,gz[deg/s] Execute with update () function
		void setspeed(float v1 = 0.f, float v2 = 0.f, float v3 = 0.f, float v4 = 0.f);								//v1~4 [rps] Execute with update () function
		void setspeed_rpm(float v1 = 0.f, float v2 = 0.f, float v3 = 0.f, float v4 = 0.f);							//v1~4 [rpm] Execute with update () function
		void setangular(float w1 = 0.f, float w2 = 0.f, float w3 = 0.f, float w4 = 0.f);							//w1~4 [rad/s] Execute with update () function
		//Update
		void update(); //Run regularly
		//Posture
		inline float pitch(float unit = ROBOT_DEG)
		{
			return (Pitch-ini_Pitch) * unit;
		}
		inline float roll(float unit = ROBOT_DEG)
		{
			return (Roll-ini_Roll) * unit;
		}
		inline float yaw(float unit = ROBOT_DEG)
		{
			return (Yaw-ini_Yaw) * unit;
		}
		//Coordinate
		inline float x(float unit = ROBOT_mm)
		{
			return X * unit;
		}
		inline float y(float unit = ROBOT_mm)
		{
			return Y * unit;
		}
		inline float wyaw(float unit = ROBOT_DEG)
		{
			return wYaw * unit;
		}
		//velocity
		inline float Rx(float unit = ROBOT_mm)
		{
			return rx * unit;
		}
		inline float Ry(float unit = ROBOT_mm)
		{
			return ry * unit;
		}
		inline float Angular(float unit = ROBOT_DEG)
		{
			return angular * unit;
		}
		inline float V(float unit = ROBOT_m)
		{
			return sqrtf(rx * ry) * unit;
		}
		//Other
		inline odmmode Mode()
		{
			return mode;
		}
		inline float DeltaT()//Minute time(Update interval)
		{
			return dt;
		}

		void reset();
};

#if defined __cplusplus
}
#endif
#endif