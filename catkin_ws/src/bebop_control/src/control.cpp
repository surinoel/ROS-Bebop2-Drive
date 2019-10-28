#include <iostream>
#include <cstdio>
#include <fstream>	// std::ifstream
#include <sstream>
#include <math.h>

#include "control.h"
#include "kbhit.h"
#include "print.h"
#include "GPStoXYDistance.h"

#define TAKEOFF 0
#define LAND 1
#define EMERGENCY 2
#define MOVE 3

#define dt 0.001
#define TOLERANCE 4
#define TOLERANCEYAW 3

#define maxSpeedX 0.5
#define maxSpeedY 0.5
#define maxSpeedZ 0.5
#define maxSpeedYaw 0.3

#define KP_X 1
#define KI_X 1
#define KD_X 1

#define KP_Y 1
#define KI_Y 1
#define KD_Y 1

#define KP_Z 1
#define KI_Z 1
#define KD_Z 1

#define KP_Yaw 1
#define KI_Yaw 1
#define KD_Yaw 1

CONTROL *bebop_control;

// ready to publish commands of takeoff, land, move control
void CONTROL::readyToPublish(ros::NodeHandle& nh){
	ROS_INFO("ready to publish");
	pub_[TAKEOFF] = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
	pub_[LAND] = nh.advertise<std_msgs::Empty>("/bebop/land", 1);
	pub_[EMERGENCY] = nh.advertise<std_msgs::Empty>("/bebop/reset", 1);
	pub_[MOVE] = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
}

// execute commands of takeoff, land, move control
void CONTROL::executeCommand(ros::Publisher pub){
	ROS_INFO("execute a command");
	std_msgs::Empty command;
	pub.publish(command);
}

/*
<REFERENCE>
Alternatively, you could define a generic queue object with a callback, and pass the object to the subscriber:
MagicQueue magicQueue1, magicQueue2;
ros::Subscriber sub = n.subscribe("topic1",100,&MagicQueue::callback,magicQueue1);
ros::Subscriber sub = n.subscribe("topic2",100,&MagicQueue::callback,magicQueue2);
*/
void CONTROL::subscribers(ros::NodeHandle& nh){
	ROS_INFO("subscribers");
	subscribeHome = nh.subscribe("/bebop/fix", 100, &CONTROL::saveHomeGPS, this);
	subscribeGPS = nh.subscribe("/bebop/states/ardrone3/PilotingState/PositionChanged", 100, &CONTROL::subGPSData, this);
	subscribeRotation = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 100, &CONTROL::subRotationData, this);
}

void CONTROL::saveHomeGPS(const sensor_msgs::NavSatFix::ConstPtr& homeGPS){
	ROS_INFO("subscribers callback function 'saveHomeGPS'");
	homeGPSData.longitude = homeGPS->longitude;			// x
	homeGPSData.latitude = homeGPS->latitude;				// y
	homeGPSData.altitude = homeGPS->altitude + 1;		// '0' is saved to altitude during takeoff

	printHomeGPS(homeGPSData.longitude, homeGPSData.latitude, homeGPSData.altitude);

	subscribeHome.shutdown();		// close 'subscribeHome' of subscribe
}

void CONTROL::subGPSData(const bebop_msgs::Ardrone3PilotingStatePositionChanged::ConstPtr& droneGPS){
	ROS_INFO("subscribers callback function 'subGPSData'");
	droneGPSData.longitude = droneGPS->longitude;			// x
	droneGPSData.latitude = droneGPS->latitude;				// y
	droneGPSData.altitude = droneGPS->altitude;

	printDroneGPS(droneGPSData.longitude, droneGPSData.latitude, droneGPSData.altitude);
}

void CONTROL::subRotationData(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr& droneRotation){
	ROS_INFO("subscribers callback function 'subRotationData'");
	droneRotationData.roll = droneRotation->roll;
	droneRotationData.pitch = droneRotation->pitch;
	droneRotationData.yaw = droneRotation->yaw;

	printRotation(droneRotationData.roll, droneRotationData.pitch, droneRotationData.yaw);
}

void CONTROL::returnHome(ros::Publisher& pub){
	ROS_INFO("return home");
	double latitude_gap, longitude_gap, altitude_gap;
	double theta;
	double speedX, speedY;
	double preSpeedX = 0, preSpeedY = 0;
	double preAltitude_gap = 0;
	double pX, iX, dX;
	double pY, iY, dY;
	double pZ, iZ, dZ;
	double pYaw, iYaw, dYaw;

	ros::Rate loopRate(100);

	while(calDistance(droneGPSData.latitude, droneGPSData.longitude, homeGPSData.latitude, homeGPSData.longitude) >= TOLERANCE){
		ros::spinOnce();	// subscribe all sub data

		longitude_gap = homeGPSData.longitude - droneGPSData.longitude;	// distance from current position to home position
		latitude_gap = homeGPSData.latitude - droneGPSData.latitude;
		altitude_gap = homeGPSData.altitude - droneGPSData.altitude;

		if(droneRotationData.yaw >= -1.5 && droneRotationData.yaw <= 3.)
			theta = 1.5 - droneRotationData.yaw;	// radian between x axis and drone' x vector
		else
			theta = -4.5 - droneRotationData.yaw;

		printGap(calDistance(droneGPSData.latitude, droneGPSData.longitude, homeGPSData.latitude, homeGPSData.longitude), longitude_gap, latitude_gap, altitude_gap, theta);

		//speedX = -(longitude_gap * sin(theta + degreeToRadian(90))) + (latitude_gap * cos(theta + degreeToRadian(90)));
		speedX = (longitude_gap * cos(theta)) + (latitude_gap * sin(theta));
		speedY = -(longitude_gap * sin(theta)) + (latitude_gap * cos(theta));
/*
		pX = KP_X * speedX;
		iX = iX + (KI_X * speedX * dt);
		dX = KD_X * (speedX - preSpeedX) / dt;

		pY = KP_Y * speedY;
		iY = iY + (KI_Y * speedY * dt);
		dY = KD_Y * (speedY - preSpeedY) / dt;

		pZ = KP_Z * altitude_gap;
		iZ = iZ + (KI_Z * altitude_gap * dt);
		dZ = KD_Z * (altitude_gap - preAltitude_gap) / dt;

		preSpeedX = speedX;
		preSpeedY = speedY;

		droneSpeed.linear.x = pX + iX + dX;
		droneSpeed.linear.y = pY + iY + dY;
		droneSpeed.linear.z = 0;
*/

		droneSpeed.linear.x = speedX / dt;
		droneSpeed.linear.y = speedY / dt;
		droneSpeed.linear.z = altitude_gap * dt;

		droneSpeed.angular.x = droneSpeed.angular.y = 0;
		droneSpeed.angular.z = 0;

		printSpeed(droneSpeed.linear.x, droneSpeed.linear.y, droneSpeed.linear.z, droneSpeed.angular.z);
		printHomeGPS(homeGPSData.longitude, homeGPSData.latitude, homeGPSData.altitude);
		printDroneGPS(droneGPSData.longitude, droneGPSData.latitude, droneGPSData.altitude);

		pub.publish(droneSpeed);
		loopRate.sleep();
	}
}

double degreeToRadian(double degree){
	return degree * (M_PI / 180);
}

double radianToDegree(double radian){
	return radian * (180 / M_PI);
}

double latitudeToX(double distance, double latitude, double longitude){
	return distance * cos(latitude) * cos(longitude);
}

double longtitudeToY(double distance, double latitude, double longitude){
	return distance * sin(latitude) * cos(longitude);
}

/*
<SPHERICAL COORDINATES TO RECTANGULAR COORDINATES>
x = r * cos(a) * cos(b)
y = r * sin(a) * cos(b)
z = r * sin(b)

r = sqrt(x* x + y * y + z * z)
a = arctan(y / x)
b = arcsin(z / r)
*/

void CONTROL::controlGPS(ros::Publisher& pub){
	ROS_INFO("control GPS");
	double longitude_gap, latitude_gap, altitude_gap, yaw_gap;
	double theta;
	double speedX, speedY, speedYaw;
	double preSpeedX = 0, preSpeedY = 0;
	double preAltitude_gap = 0, preYaw_gap = 0;
	double pX, iX, dX;
	double pY, iY, dY;
	double pZ, iZ, dZ;
	double pYaw, iYaw, dYaw;

	ros::Rate loopRate(100);

	while(calDistance(droneGPSData.latitude, droneGPSData.longitude, homeGPSData.latitude, homeGPSData.longitude) >= TOLERANCE || abs(radianToDegree(yaw_gap)) >= TOLERANCEYAW){	// distance fronm drone' position to destination

		longitude_gap = homeGPSData.longitude - droneGPSData.longitude;
		latitude_gap = homeGPSData.latitude - droneGPSData.latitude;
		altitude_gap = homeGPSData.altitude - droneGPSData.altitude;

		if(droneRotationData.yaw >= -1.5 && droneRotationData.yaw <= 3.)
			theta = 1.5 - droneRotationData.yaw;	// radian between x axis and drone' x vector
		else
			theta = -4.5 - droneRotationData.yaw;

		//speedX = -(longitude_gap * sin(theta + degreeToRadian(90))) + (latitude_gap * cos(theta + degreeToRadian(90)));
		speedX = (longitude_gap * cos(theta)) + (latitude_gap * sin(theta));
		speedY = -(longitude_gap * sin(theta)) + (latitude_gap * cos(theta));
		yaw_gap = atan2(speedY, speedX);
/*
		pX = KP_X * speedX;
		iX = iX + (KI_X * speedX * dt);
		dX = KD_X * (speedX - preSpeedX) / dt;

		pY = KP_Y * speedY;
		iY = iY + (KI_Y * speedY * dt);
		dY = KD_Y * (speedY - preSpeedY) / dt;

		pZ = KP_Z * altitude_gap;
		iZ = iZ + (KI_Z * altitude_gap * dt);
		dZ = KD_Z * (altitude_gap - preAltitude_gap) / dt;

		pYaw = KP_Yaw * altitude_gap;
		iYaw = iYaw + (KI_Yaw * altitude_gap * dt);
		dYaw = KD_Yaw * (yaw_gap - preYaw_gap) / dt;

		preSpeedX = speedX;
		preSpeedY = speedY;
		preAltitude_gap = altitude_gap;
		preYaw_gap = yaw_gap;
*/
		if(speedY >= 0.){
				speedYaw = maxSpeedYaw;
				ROS_INFO("y >= 0");
		}
		else{
				speedYaw = -maxSpeedYaw;
				ROS_INFO("y < 0");
		}

		printHomeGPS(homeGPSData.longitude, homeGPSData.latitude, homeGPSData.altitude);
		printDroneGPS(droneGPSData.longitude, droneGPSData.latitude, droneGPSData.altitude);
		printRotation(droneRotationData.roll, droneRotationData.pitch, droneRotationData.yaw);
		printGap(calDistance(droneGPSData.latitude, droneGPSData.longitude, homeGPSData.latitude, homeGPSData.longitude), longitude_gap, latitude_gap, altitude_gap, yaw_gap);
		ROS_INFO("[GAP] YAWGAP : %lf", radianToDegree(yaw_gap));

		// move to destination
		droneSpeed.linear.x = speedX / dt;	//pX + iX + dX;
		if (droneSpeed.linear.x > maxSpeedX) droneSpeed.linear.x = maxSpeedX;				// define the max speed of x
		else if (droneSpeed.linear.x < -maxSpeedX) droneSpeed.linear.x = -maxSpeedX;
		droneSpeed.linear.y = speedY / dt;	// pY + iY + dY;
		if (droneSpeed.linear.y > maxSpeedY) droneSpeed.linear.y = maxSpeedY;				// define the max speed of y
		else if (droneSpeed.linear.y < -maxSpeedY) droneSpeed.linear.y = -maxSpeedY;
		droneSpeed.linear.z = altitude_gap * dt;	// Z + iZ + dZ;
		if (droneSpeed.linear.z > maxSpeedZ) droneSpeed.linear.z = maxSpeedZ;				// define the max speed of z
		else if (droneSpeed.linear.z < -maxSpeedZ) droneSpeed.linear.z = -maxSpeedZ;
		droneSpeed.angular.z = speedYaw;	//Yaw + iYaw + dYaw;
		if (droneSpeed.angular.z > maxSpeedYaw) droneSpeed.angular.z = maxSpeedYaw;	// define the max speed of yaw
		else if (droneSpeed.angular.z < -maxSpeedYaw) droneSpeed.angular.z = -maxSpeedYaw;

		droneSpeed.angular.x = droneSpeed.angular.y = 0;

		printSpeed(droneSpeed.linear.x, droneSpeed.linear.y, droneSpeed.linear.z, droneSpeed.angular.z);

		pub.publish(droneSpeed);
		ros::spinOnce();	// subscribe all sub data
		loopRate.sleep();
	}
}

void CONTROL::controlGPS_yawAndMove(ros::Publisher& pub){
	ROS_INFO("control GPS_yaw first");
	double longitude_gap, latitude_gap, altitude_gap, yaw_gap = TOLERANCEYAW;
	double theta;
	double speedX, speedY, speedYaw;
	double preSpeedX = 0, preSpeedY = 0;
	double preAltitude_gap = 0, preYaw_gap = 0;
	double pX, iX, dX;
	double pY, iY, dY;
	double pZ, iZ, dZ;
	double pYaw, iYaw, dYaw;
	double distance;

	bool yawFlag = false;

	ros::Rate loopRate(50);

	while(calDistance(droneGPSData.latitude, droneGPSData.longitude, goalGPSData.latitude, goalGPSData.longitude) >= TOLERANCE){
		while(yawFlag == false && abs(radianToDegree(yaw_gap)) >= TOLERANCEYAW){

			longitude_gap = goalGPSData.longitude - droneGPSData.longitude;
			latitude_gap = goalGPSData.latitude - droneGPSData.latitude;
			altitude_gap = goalGPSData.altitude - droneGPSData.altitude;

			if(droneRotationData.yaw >= -1.5 && droneRotationData.yaw <= 3.)
				theta = 1.5 - droneRotationData.yaw;	// radian between x axis and drone' x vector
			else
				theta = -4.5 - droneRotationData.yaw;

			speedX = (longitude_gap * cos(theta)) + (latitude_gap * sin(theta));
			speedY = -(longitude_gap * sin(theta)) + (latitude_gap * cos(theta));
			yaw_gap = atan2(speedY, speedX);

			ROS_INFO("---------------------------------------");
			ROS_INFO("[THETA] THETA : %lf", theta);
			ROS_INFO("[DRONE] YAW : %lf", droneRotationData.yaw);
			ROS_INFO("[GAP] YAW_GAP : %lf", yaw_gap);
			ROS_INFO("[GAP] YAW_GAP(DEGREE) : %lf", radianToDegree(yaw_gap));
			ROS_INFO("[SPEED] X : %lf", speedX);
			ROS_INFO("[SPEED] Y : %lf", speedY);
/*
			if(speedY >= 0.){
					speedYaw = maxSpeedYaw;
					ROS_INFO("y >= 0");
			}
			else{
					speedYaw = -maxSpeedYaw;
					ROS_INFO("y < 0");
			}
*/
			droneSpeed.linear.x = droneSpeed.linear.y = droneSpeed.linear.z = 0;
			droneSpeed.angular.x = droneSpeed.angular.y = 0;
			droneSpeed.angular.z = yaw_gap / 1.5;

			ROS_INFO("[SPEED] YAW : %lf", droneSpeed.angular.z);
			ROS_INFO("---------------------------------------");

			pub.publish(droneSpeed);
			ros::spinOnce();
			loopRate.sleep();
		}
		distance = calDistance(droneGPSData.latitude, droneGPSData.longitude, goalGPSData.latitude, goalGPSData.longitude);
		yawFlag = true;

		longitude_gap = goalGPSData.longitude - droneGPSData.longitude;
		latitude_gap = goalGPSData.latitude - droneGPSData.latitude;
		altitude_gap = goalGPSData.altitude - droneGPSData.altitude;

		if(droneRotationData.yaw >= -1.5 && droneRotationData.yaw <= 3.)
			theta = 1.5 - droneRotationData.yaw;	// radian between x axis and drone' x vector
		else
			theta = -4.5 - droneRotationData.yaw;

		speedX = (longitude_gap * cos(theta)) + (latitude_gap * sin(theta));
		speedY = -(longitude_gap * sin(theta)) + (latitude_gap * cos(theta));
		yaw_gap = atan2(speedY, speedX);
/*
		pX = KP_X * speedX;
		iX = iX + (KI_X * speedX * dt);
		dX = KD_X * (speedX - preSpeedX) / dt;

		pY = KP_Y * speedY;
		iY = iY + (KI_Y * speedY * dt);
		dY = KD_Y * (speedY - preSpeedY) / dt;

		pZ = KP_Z * altitude_gap;
		iZ = iZ + (KI_Z * altitude_gap * dt);
		dZ = KD_Z * (altitude_gap - preAltitude_gap) / dt;

		pYaw = KP_Yaw * altitude_gap;
		iYaw = iYaw + (KI_Yaw * altitude_gap * dt);
		dYaw = KD_Yaw * (yaw_gap - preYaw_gap) / dt;

		preSpeedX = speedX;
		preSpeedY = speedY;
		preAltitude_gap = altitude_gap;
		preYaw_gap = yaw_gap;
*/
		droneSpeed.linear.x = distance / 20.;	// pX + iX + dX;
		if (droneSpeed.linear.x > maxSpeedX) droneSpeed.linear.x = maxSpeedX;				// define the max speed of x
		else if (droneSpeed.linear.x < -maxSpeedX) droneSpeed.linear.x = -maxSpeedX;
		droneSpeed.linear.y = speedY / dt;	// pY + iY + dY;
		if (droneSpeed.linear.y > maxSpeedY) droneSpeed.linear.y = maxSpeedY;				// define the max speed of y
		else if (droneSpeed.linear.y < -maxSpeedY) droneSpeed.linear.y = -maxSpeedY;
		droneSpeed.linear.z = altitude_gap * dt;	//pZ + iZ + dZ;
		if (droneSpeed.linear.z > maxSpeedZ) droneSpeed.linear.z = maxSpeedZ;				// define the max speed of z
		else if (droneSpeed.linear.z < -maxSpeedZ) droneSpeed.linear.z = -maxSpeedZ;
		droneSpeed.angular.z = yaw_gap;	// pYaw + iYaw + dYaw;
		if (droneSpeed.angular.z > maxSpeedYaw) droneSpeed.angular.z = maxSpeedYaw;	// define the max speed of yaw
		else if (droneSpeed.angular.z < -maxSpeedYaw) droneSpeed.angular.z = -maxSpeedYaw;

		droneSpeed.angular.x = droneSpeed.angular.y = 0;

		ROS_INFO("---------------------------------------");
		printDroneGPS(droneGPSData.longitude, droneGPSData.latitude, droneGPSData.altitude);
		printDestinationGPS(goalGPSData.longitude, goalGPSData.latitude, goalGPSData.altitude);
		printGap(distance, longitude_gap, latitude_gap, altitude_gap, yaw_gap);
		printSpeed(droneSpeed.linear.x, droneSpeed.linear.y, droneSpeed.linear.z, droneSpeed.angular.z);
		ROS_INFO("---------------------------------------");

		pub.publish(droneSpeed);
		ros::spinOnce();	// subscribe all sub data
		loopRate.sleep();
	}

	droneSpeed.linear.x = droneSpeed.linear.y = droneSpeed.angular.z = 0;
	pub.publish(droneSpeed);
}

void CONTROL::getGPSDataOfGoal(){
	std::string destination;
	std::cout << "Input destination : ";
	//destination = "101";
	// std::getline(std::cin, destination);
	std::cin >> destination;

	std::ifstream goalGPS("/home/nim/catkin_ws/src/bebop_control/data/gpsData.txt");

	if(!goalGPS.is_open()){	// whether file is open
		std::cout << "not open\n";
	}

	std::string buf;
	float dataOfGPS[4];
	while(goalGPS){
		getline(goalGPS, buf);
		if(buf.find(destination) != std::string::npos){
			std::istringstream gpsData(buf);
			gpsData >> dataOfGPS[0] >> dataOfGPS[1] >> dataOfGPS[2] >> dataOfGPS[3];
			break;
		}
	}
	for(int i = 0; i < 4; ++i)
		std::cout << dataOfGPS[i] << "\n";

	goalGPSData.longitude = dataOfGPS[1];
	goalGPSData.latitude = dataOfGPS[2];
	goalGPSData.altitude = dataOfGPS[3];
}

// control the drone with keyboard
void controlKeyboard(const ros::TimerEvent& timerEventKey){
	init_keyboard();
	if(_kbhit()){
		int ch = _getch();
		_putch(ch);
		switch (ch) {
			case '1':
				ROS_INFO("take off");
				bebop_control->executeCommand(bebop_control->pub_[TAKEOFF]);
				break;
			case '2':
				ROS_INFO("land");
				bebop_control->executeCommand(bebop_control->pub_[LAND]);
				break;
			case '3':
				ROS_INFO("emergency landing");
				bebop_control->executeCommand(bebop_control->pub_[EMERGENCY]);
				break;
			case '4':
				ROS_INFO("return home");
				bebop_control->returnHome(bebop_control->pub_[MOVE]);
				break;
			case '5':
				ROS_INFO("go to destination");
				bebop_control->controlGPS(bebop_control->pub_[MOVE]);
				break;
			case '6':
				ROS_INFO("go to destination");
				bebop_control->getGPSDataOfGoal();
				bebop_control->controlGPS_yawAndMove(bebop_control->pub_[MOVE]);
				break;

			case 'i':			// drone is moving during pushing a key
			case 'I':
				ROS_INFO("forward");
				bebop_control->droneSpeed.linear.x = 1;
				bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
			case 'k':			// drone is moving during pushing a key
			case 'K':
				ROS_INFO("backward");
				bebop_control->droneSpeed.linear.x = -1;
				bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
			case 'j':			// drone is moving during pushing a key
			case 'J':
				ROS_INFO("left");
				bebop_control->droneSpeed.linear.y = 1;
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
			case 'l':			// drone is moving during pushing a key
			case 'L':
				ROS_INFO("right");
				bebop_control->droneSpeed.linear.y = -1;
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
			case 'w':			// drone is moving during pushing a key
			case 'W':
				ROS_INFO("up");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = 0;
				bebop_control->droneSpeed.linear.z = 1;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
			case 's':			// drone is moving during pushing a key
			case 'S':
				ROS_INFO("down");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = 0;
				bebop_control->droneSpeed.linear.z = -1;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
			case 'a':			// drone is moving during pushing a key
			case 'A':
				ROS_INFO("yaw_left");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = 0;
				bebop_control->droneSpeed.angular.z = 1;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
			case 'd':			// drone is moving during pushing a key
			case 'D':
				ROS_INFO("yaw_right");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = 0;
				bebop_control->droneSpeed.angular.z = -1;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
			default:
			ROS_INFO("yaw_right");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
				break;
		}
	}
}
