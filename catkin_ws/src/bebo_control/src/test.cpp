#include <math.h>

#include "test.h"
#include "kbhit.h"
#include "GPStoXYDistance.h"

#define TAKEOFF 0
#define LAND 1
#define EMERGENCY 2
#define MOVE 3

#define dt 0.0001
#define TOLERANCE 0.5

#define maxSpeedX 1
#define maxSpeedY 1
#define maxSpeedZ 0.5
#define maxSpeedYaw 0.3

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

	ROS_INFO("[HOME] LONGITUDE : %lf", homeGPSData.longitude);
	ROS_INFO("[HOME] LATITUDE : %lf", homeGPSData.latitude);
	ROS_INFO("[HOME] ALTITUDE : %lf", homeGPSData.altitude);

	subscribeHome.shutdown();		// close 'subscribeHome' of subscribe
}

void CONTROL::subGPSData(const bebop_msgs::Ardrone3PilotingStatePositionChanged::ConstPtr& droneGPS){
	ROS_INFO("subscribers callback function 'subGPSData'");
	droneGPSData.longitude = droneGPS->longitude;			// x
	droneGPSData.latitude = droneGPS->latitude;				// y
	droneGPSData.altitude = droneGPS->altitude;

	ROS_INFO("[DRONE] LONGITUDE : %lf", droneGPSData.longitude);
	ROS_INFO("[DRONE] LATITUDE : %lf", droneGPSData.latitude);
	ROS_INFO("[DRONE] ALTITUDE : %lf", droneGPSData.altitude);
}

void CONTROL::subRotationData(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr& droneRotation){
	ROS_INFO("subscribers callback function 'subRotationData'");
	droneRotationData.roll = droneRotation->roll;
	droneRotationData.pitch = droneRotation->pitch;
	droneRotationData.yaw = droneRotation->yaw;

	ROS_INFO("[DRONE] ROLL : %lf", droneRotationData.roll);
	ROS_INFO("[DRONE] PITCH : %lf", droneRotationData.pitch);
	ROS_INFO("[DRONE] YAW : %lf", droneRotationData.yaw);
}

void CONTROL::returnHome(ros::Publisher& pub){
	ROS_INFO("return home");
	double latitude_gap, longitude_gap, altitude_gap;
	double theta;
	double speedX, speedY;

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

		ROS_INFO("[DISTANCE] %lf", calDistance(droneGPSData.latitude, droneGPSData.longitude, homeGPSData.latitude, homeGPSData.longitude));
		ROS_INFO("[GAP] XGAP : %lf", longitude_gap);
		ROS_INFO("[GAP] YGAP : %lf", latitude_gap);
		ROS_INFO("[GAP] ZGAP : %lf", altitude_gap);
		ROS_INFO("[GAP] THATA : %lf", theta);

		//speedX = -(longitude_gap * sin(theta + degreeToRadian(90))) + (latitude_gap * cos(theta + degreeToRadian(90)));
		speedX = (longitude_gap * cos(theta)) + (latitude_gap * sin(theta));
		speedY = -(longitude_gap * sin(theta)) + (latitude_gap * cos(theta));

		droneSpeed.linear.x = speedX / dt;
		droneSpeed.linear.y = speedY / dt;
		droneSpeed.linear.z = altitude_gap * dt;

		droneSpeed.angular.x = droneSpeed.angular.y = 0;
		droneSpeed.angular.z = 0;

		ROS_INFO("[SPEED] X : %lf", droneSpeed.linear.x);
		ROS_INFO("[SPEED] Y : %lf", droneSpeed.linear.y);
		ROS_INFO("[SPEED] Z : %lf", droneSpeed.linear.z);
		ROS_INFO("[SPEED] YAW : %lf", droneSpeed.angular.z);
		ROS_INFO("[HOME] LONGITUDE : %lf", homeGPSData.longitude);
		ROS_INFO("[HOME] LATITUDE : %lf", homeGPSData.latitude);
		ROS_INFO("[HOME] ALTITUDE : %lf", homeGPSData.altitude);
		ROS_INFO("[DRONE] LONGITUDE : %lf", droneGPSData.longitude);
		ROS_INFO("[DRONE] LATITUDE : %lf", droneGPSData.latitude);
		ROS_INFO("[DRONE] ALTITUDE : %lf", droneGPSData.altitude);

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
	double longitude_gap, latitude_gap, altitude_gap, yaw_gap = 1.;
	double theta;
	double speedX, speedY;

	ros::Rate loopRate(100);

	while(calDistance(droneGPSData.latitude, droneGPSData.longitude, homeGPSData.latitude, homeGPSData.longitude) >= TOLERANCE || abs(radianToDegree(yaw_gap)) >= 5.){	// distance fronm drone' position to destination

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

		if(speedX >= 0. && speedY >= 0.){
				yaw_gap = atan2(speedY, speedX);
				ROS_INFO("x >= 0, y >= 0");
		}
		else if(speedX < 0. && speedY >= 0.){
				yaw_gap = -1.5 - atan2(speedX, speedY);
				ROS_INFO("x < 0, y >= 0");
		}
		else if(speedX >= 0. && speedY < 0.){
				yaw_gap = atan2(speedY, speedX);
				ROS_INFO("x >= 0, y < 0");
		}
		else{
				yaw_gap = 1.5 + atan2(speedX, speedY);
				ROS_INFO("x < 0, y < 0");
		}

		ROS_INFO("[HOME] LONGITUDE : %lf", homeGPSData.longitude);
		ROS_INFO("[HOME] LATITUDE : %lf", homeGPSData.latitude);
		ROS_INFO("[HOME] ALTITUDE : %lf", homeGPSData.altitude);
		ROS_INFO("[DRONE] LONGITUDE : %lf", droneGPSData.longitude);
		ROS_INFO("[DRONE] LATITUDE : %lf", droneGPSData.latitude);
		ROS_INFO("[DRONE] ALTITUDE : %lf", droneGPSData.altitude);
		ROS_INFO("[DRONE] YAW : %lf",droneRotationData.yaw);
		ROS_INFO("[GAP] DISTANCE : %lf", calDistance(droneGPSData.latitude, droneGPSData.longitude, homeGPSData.latitude, homeGPSData.longitude));
		ROS_INFO("[GAP] XGAP : %lf", longitude_gap);
		ROS_INFO("[GAP] YGAP : %lf", latitude_gap);
		ROS_INFO("[GAP] ZGAP : %lf", altitude_gap);
		ROS_INFO("[GAP] YAWGAP : %lf", yaw_gap);
		ROS_INFO("[GAP] YAWGAP : %lf", radianToDegree(yaw_gap));

		// move to destination
		droneSpeed.linear.x = speedX / dt;
		if (droneSpeed.linear.x > maxSpeedX) droneSpeed.linear.x = maxSpeedX;				// define the max speed of x
		else if (droneSpeed.linear.x < -maxSpeedX) droneSpeed.linear.x = -maxSpeedX;
		droneSpeed.linear.y = speedY / dt;
		if (droneSpeed.linear.y > maxSpeedY) droneSpeed.linear.y = maxSpeedY;				// define the max speed of y
		else if (droneSpeed.linear.y < -maxSpeedY) droneSpeed.linear.y = -maxSpeedY;
		droneSpeed.linear.z = altitude_gap * dt;
		if (droneSpeed.linear.z > maxSpeedZ) droneSpeed.linear.z = maxSpeedZ;				// define the max speed of z
		else if (droneSpeed.linear.z < -maxSpeedZ) droneSpeed.linear.z = -maxSpeedZ;
		droneSpeed.angular.z = yaw_gap;
		if (droneSpeed.angular.z > maxSpeedYaw) droneSpeed.angular.z = maxSpeedYaw;	// define the max speed of yaw
		else if (droneSpeed.angular.z < -maxSpeedYaw) droneSpeed.angular.z = -maxSpeedYaw;

		droneSpeed.angular.x = droneSpeed.angular.y = 0;

		ROS_INFO("[SPEED] X : %lf", droneSpeed.linear.x);
		ROS_INFO("[SPEED] Y : %lf", droneSpeed.linear.y);
		ROS_INFO("[SPEED] Z : %lf", droneSpeed.linear.z);
		ROS_INFO("[SPEED] YAW : %lf", droneSpeed.angular.z);

		pub.publish(droneSpeed);
		ros::spinOnce();	// subscribe all sub data
		loopRate.sleep();
	}
}

void CONTROL::controlGPS_yawAndMove(ros::Publisher& pub){
	ROS_INFO("control GPS_yaw first");
	double longitude_gap, latitude_gap, altitude_gap, yaw_gap = 1.;
	double theta;
	double speedX, speedY, speedYaw;

	ros::Rate loopRate(50);

	while(abs(radianToDegree(yaw_gap)) >= 5){

		longitude_gap = homeGPSData.longitude - droneGPSData.longitude;
		latitude_gap = homeGPSData.latitude - droneGPSData.latitude;
		altitude_gap = homeGPSData.altitude - droneGPSData.altitude;

		if(droneRotationData.yaw >= -1.5 && droneRotationData.yaw <= 3.)
			theta = 1.5 - droneRotationData.yaw;	// radian between x axis and drone' x vector
		else
			theta = -4.5 - droneRotationData.yaw;

		speedX = (longitude_gap * cos(theta)) + (latitude_gap * sin(theta));
		speedY = -(longitude_gap * sin(theta)) + (latitude_gap * cos(theta));
		yaw_gap = atan2(speedY, speedX);

		ROS_INFO("---------------------------------------");
		ROS_INFO("[THETA] THETA : %lf", theta);
		ROS_INFO("[DRONE] YAW : %lf",droneRotationData.yaw);
		ROS_INFO("[GAP] YAW_GAP : %lf", yaw_gap);
		ROS_INFO("[GAP] YAW_GAP(DEGREE) : %lf", radianToDegree(yaw_gap));
		ROS_INFO("[SPEED] X : %lf", speedX);
		ROS_INFO("[SPEED] Y : %lf", speedY);

		if(speedY >= 0.){
				speedYaw = maxSpeedYaw;
				ROS_INFO("y >= 0");
		}
		else{
				speedYaw = -maxSpeedYaw;
				ROS_INFO("y < 0");
		}

		droneSpeed.linear.x = droneSpeed.linear.y = droneSpeed.linear.z = 0;
		droneSpeed.angular.x = droneSpeed.angular.y = 0;
		droneSpeed.angular.z = speedYaw;

		ROS_INFO("[SPEED] YAW : %lf", droneSpeed.angular.z);
		ROS_INFO("---------------------------------------");

		pub.publish(droneSpeed);
		ros::spinOnce();
		loopRate.sleep();
	}

	while(calDistance(droneGPSData.latitude, droneGPSData.longitude, homeGPSData.latitude, homeGPSData.longitude) >= TOLERANCE || abs(radianToDegree(yaw_gap)) >= 5.){
		longitude_gap = homeGPSData.longitude - droneGPSData.longitude;
		latitude_gap = homeGPSData.latitude - droneGPSData.latitude;
		altitude_gap = homeGPSData.altitude - droneGPSData.altitude;

		if(droneRotationData.yaw >= -1.5 && droneRotationData.yaw <= 3.)
			theta = 1.5 - droneRotationData.yaw;	// radian between x axis and drone' x vector
		else
			theta = -4.5 - droneRotationData.yaw;

		speedX = (longitude_gap * cos(theta)) + (latitude_gap * sin(theta));
		speedY = -(longitude_gap * sin(theta)) + (latitude_gap * cos(theta));
		yaw_gap = atan2(speedY, speedX);

		droneSpeed.linear.x = speedX / dt;
		if (droneSpeed.linear.x > maxSpeedX) droneSpeed.linear.x = maxSpeedX;				// define the max speed of x
		else if (droneSpeed.linear.x < -maxSpeedX) droneSpeed.linear.x = -maxSpeedX;
		droneSpeed.linear.y = speedY / dt;
		if (droneSpeed.linear.y > maxSpeedY) droneSpeed.linear.y = maxSpeedY;				// define the max speed of y
		else if (droneSpeed.linear.y < -maxSpeedY) droneSpeed.linear.y = -maxSpeedY;
		droneSpeed.linear.z = altitude_gap * dt;
		if (droneSpeed.linear.z > maxSpeedZ) droneSpeed.linear.z = maxSpeedZ;				// define the max speed of z
		else if (droneSpeed.linear.z < -maxSpeedZ) droneSpeed.linear.z = -maxSpeedZ;
		droneSpeed.angular.z = yaw_gap;
		if (droneSpeed.angular.z > maxSpeedYaw) droneSpeed.angular.z = maxSpeedYaw;	// define the max speed of yaw
		else if (droneSpeed.angular.z < -maxSpeedYaw) droneSpeed.angular.z = -maxSpeedYaw;

		droneSpeed.angular.x = droneSpeed.angular.y = 0;

		pub.publish(droneSpeed);
		ros::spinOnce();	// subscribe all sub data
		loopRate.sleep();
	}
}

// control the drone with keyboard
void controlKeyboard(const ros::TimerEvent& timerEventKey){
	init_keyboard();
	if(_kbhit()){
		int ch = _getch();
		_putch(ch);
		switch (ch) {
			case '1': {
				ROS_INFO("take off");
				bebop_control->executeCommand(bebop_control->pub_[TAKEOFF]);
			} break;
			case '2': {
				ROS_INFO("land");
				bebop_control->executeCommand(bebop_control->pub_[LAND]);
			} break;
			case '3': {
				ROS_INFO("emergency landing");
				bebop_control->executeCommand(bebop_control->pub_[EMERGENCY]);
			} break;
			case '4': {
				ROS_INFO("return home");
				bebop_control->returnHome(bebop_control->pub_[MOVE]);
			} break;
			case '5': {
				ROS_INFO("go to destination");
				bebop_control->controlGPS(bebop_control->pub_[MOVE]);
			} break;
			case '6': {
				ROS_INFO("go to destination");
				bebop_control->controlGPS_yawAndMove(bebop_control->pub_[MOVE]);
			} break;

			case 'i':			// drone is moving during pushing a key
			case 'I': {
				ROS_INFO("forward");
				bebop_control->droneSpeed.linear.x = 1;
				bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
			} break;
			case 'k':			// drone is moving during pushing a key
			case 'K': {
				ROS_INFO("backward");
				bebop_control->droneSpeed.linear.x = -1;
				bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
			} break;
			case 'j':			// drone is moving during pushing a key
			case 'J': {
				ROS_INFO("left");
				bebop_control->droneSpeed.linear.y = 1;
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
			} break;
			case 'l':			// drone is moving during pushing a key
			case 'L': {
				ROS_INFO("right");
				bebop_control->droneSpeed.linear.y = -1;
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
			} break;
			case 'w':			// drone is moving during pushing a key
			case 'W': {
				ROS_INFO("up");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = 0;
				bebop_control->droneSpeed.linear.z = 1;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
			} break;
			case 's':			// drone is moving during pushing a key
			case 'S': {
				ROS_INFO("down");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = 0;
				bebop_control->droneSpeed.linear.z = -1;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = bebop_control->droneSpeed.angular.z = 0;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
			} break;
			case 'a':			// drone is moving during pushing a key
			case 'A': {
				ROS_INFO("yaw_left");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = 0;
				bebop_control->droneSpeed.angular.z = 1;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
			} break;
			case 'd':			// drone is moving during pushing a key
			case 'D': {
				ROS_INFO("yaw_right");
				bebop_control->droneSpeed.linear.x = bebop_control->droneSpeed.linear.y = bebop_control->droneSpeed.linear.z = 0;
				bebop_control->droneSpeed.angular.x = bebop_control->droneSpeed.angular.y = 0;
				bebop_control->droneSpeed.angular.z = -1;

				bebop_control->pub_[MOVE].publish(bebop_control->droneSpeed);
			} break;
		}
	}
}
