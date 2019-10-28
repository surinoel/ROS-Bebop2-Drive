#ifndef TEST_H
#define TEST_H

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <bebop_msgs/Ardrone3PilotingStatePositionChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>

double degreeToRadian(double);
double radianToDegree(double);

void controlKeyboard(const ros::TimerEvent&);

class CONTROL{
private:
  ros::Subscriber subscribeHome;
  ros::Subscriber subscribeGPS;
  ros::Subscriber subscribeRotation;

  sensor_msgs::NavSatFix homeGPSData;                                   // subscribe home GPS data
  bebop_msgs::Ardrone3PilotingStatePositionChanged droneGPSData, prev_droneGPSData, goalGPSData;        // subscribe GPS data of the drone
  bebop_msgs::Ardrone3PilotingStateAttitudeChanged droneRotationData, prev_droneRotationData;   // subscribe 'roll', 'pitch', 'yaw' data of the drone

public:
  ros::Publisher pub_[4];   // publish 'takeoff', 'land', 'emergency', 'move' commands
  geometry_msgs::Twist droneSpeed, prev_droneSpeed;

  void readyToPublish(ros::NodeHandle&);
  void executeCommand(ros::Publisher);

  void subscribers(ros::NodeHandle& nh);

  // callback functions
  void saveHomeGPS(const sensor_msgs::NavSatFix::ConstPtr&);
  void subGPSData(const bebop_msgs::Ardrone3PilotingStatePositionChanged::ConstPtr&);
  void subRotationData(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr&);

  void returnHome(ros::Publisher&);
  void controlGPS(ros::Publisher&);
  void controlGPS_yawAndMove(ros::Publisher&);
  void getGPSDataOfGoal();
};

extern CONTROL *bebop_control;

#endif
