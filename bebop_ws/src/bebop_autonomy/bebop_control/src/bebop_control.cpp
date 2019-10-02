#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/NavSatFix.h>

//#include "bebop_control/bebop_control.h"

#define TAKEOFF 0
#define LAND 1
#define RESET 2
#define MOVE 3

int count;

double latitude;
double longitude;
double altitude;
double yaw;

ros::Publisher pub[4];

void homeGPS(const sensor_msgs::NavSatFix::ConstPtr &home);
void drone_GPS(const sensor_msgs::NavSatFix::ConstPtr &GPS);

std_msgs::Empty takeoffLand;
sensor_msgs::NavSatFix home_position;
sensor_msgs::NavSatFix current_position;
sensor_msgs::NavSatFix move_position;
geometry_msgs::Twist input_speed;

int main(int argc, char **argv){
	ros::init(argc, argv, "bebop_control_test");	// bebop control node initialize
	
	ros::NodeHandle nh;

	pub[TAKEOFF] = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 1);		// commend to bebop_take off
	pub[LAND] = nh.advertise<std_msgs::Empty>("/bebop/land", 1);			// commend to bebop_land
	pub[RESET] = nh.advertise<std_msgs::Empty>("/bebop/reset", 1);			// commend to bebop_emergency
	pub[MOVE] = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);	// commend to bebop_move(roll, pitch, yaw, altitude)

	ros::Subscriber sub_home = nh.subscribe("/bebop/fix", 1000, homeGPS);
	ros::Subscriber sub_GPS = nh.subscribe("/bebop/states/ardrone3/PilotingState/PositionChanged", 1000, drone_GPS);
	//ros::Subscriber sub_GPS = nh.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1000, drone_GPS);
	ros::spin();	// call the callback function
	
	ros::Duration time(0.5);
	
	while(ros::ok()){
		//pub[TAKEOFF].publish(takeoffLand);
		ROS_INFO("TAKEOFF");
		time.sleep();

		ROS_INFO("HOME");
		time.sleep();
		//pub[LAND].publish(takeoffLand);
		ROS_INFO("LAND");
		time.sleep();
	}
	
	return 0;
}

void homeGPS(const sensor_msgs::NavSatFix::ConstPtr &home){	// GPS data of home position
	ROS_INFO("[bebop_fix] latitude : %lf, longitude : %lf, altitude : %lf", home->latitude, home->longitude, home->altitude);
	
	if(count == 0){
		ROS_INFO("Get home position");
		
		/* save the home position */
		home_position.latitude = home->latitude;
		home_position.longitude = home->longitude;
		home_position.altitude = home->altitude;
		++count;
	}

	//ROS_INFO("<home> latitude : %lf, longitude : %lf, altitude : %lf", home_position.latitude, home_position.longitude, home_position.altitude);
}

void drone_GPS(const sensor_msgs::NavSatFix::ConstPtr &GPS){
	ROS_INFO("[drone_GPS] latitude : %lf, longitude : %lf, altitude : %lf", GPS->latitude, GPS->longitude, GPS->altitude);
	/* save the current position */
	current_position.latitude = GPS->latitude;
	current_position.longitude = GPS->longitude;
	current_position.altitude = GPS->altitude;
	
	double latitude_gap = latitude - current_position.latitude;
	double longitude_gap = longitude - current_position.longitude;
	double altitude_gap = altitude - current_position.altitude;
	double yaw_gap = 0;
	
	// theta range
	/*if(){
		
	}*/
	
	while(latitude_gap > 0 || longitude_gap > 0 || altitude_gap > 0){
		if(latitude_gap > 0)
			input_speed.linear.x = 2;
		else
			input_speed.linear.x = 0;
			
		if(longitude_gap > 0)
			input_speed.linear.y = 2;
		else
			input_speed.linear.y = 0;
			
		if(altitude_gap > 0)
			input_speed.linear.z = 2;
		else
			input_speed.linear.z = 0;
			
		if(yaw_gap > 0)
			input_speed.angular.z = 1;
		else
			input_speed.angular.z = 0;
	}
	 
	input_speed.linear.x = input_speed.linear.y = input_speed.linear.z = 0;
	input_speed.angular.z = 0;
}
