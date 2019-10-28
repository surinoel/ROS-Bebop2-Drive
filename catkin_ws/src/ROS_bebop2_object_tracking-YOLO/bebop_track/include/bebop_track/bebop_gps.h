#ifndef __BEBOP_GPS_H__
#define __BEBOP_GPS_H__

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include "bebop_msgs/Ardrone3PilotingStatePositionChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include <nav_msgs/Odometry.h>

const bool gpsOff = false;
const bool gpsOn = true;
const bool gpsModeOff = false;
const bool gpsModeOn = true;
const bool endGoHome = false;
const bool goEnteredcoordinates = true;
const bool endGoEnteredcoordinates = false;
/*파라미터 키 이름*/
const std::string gps_key = "/bebop/gps";
const std::string gps_mode_key = "/bebop/gpsmode";
const std::string takeoff_key = "/bebop/takeoff";
const std::string home_gps_latitude_key = "/bebop/homegps/latitude";
const std::string home_gps_longitude_key = "/bebop/homegps/longitude";
const std::string drone_gps_latitude_key = "/bebop/dronegps/latitude"; // teleop 화면에 gps latitude값을 띄우기 위한 param key
const std::string drone_gps_longitude_key = "/bebop/dronegps/longitude"; // teleop 화면에 gps longitude값을 띄우기 위한 param key
const std::string go_home_key = "/bebop/gohome";
const std::string go_entered_coordinates_key = "/bebop/goenteredcoordinates";
const std::string go_latitude_key = "/bebop/go/latitude";
const std::string go_longitude_key = "/bebop/go/longitude";
/*subscribe하는 topic 이름*/
const std::string drone_gps = "/bebop/states/ardrone3/PilotingState/PositionChanged";
const std::string bebop_odom = "/bebop/odom";
const std::string bebop_attitude = "/bebop/states/ardrone3/PilotingState/AttitudeChanged";

class BebopGlobalPositioningSystem
{
private:
    ros::NodeHandle _nodeHandle;
    ros::Publisher _bebopControlFromGPSPublisher;//목표 위치와 현재 위치를 비교해서 x방향 선속도를 pub
    ros::Publisher _bebopGoHomePublisher;//home 방향으로 기체 머리를 돌리고 고도를 맞추는 pub
    ros::Subscriber _currentGPSSubscriber;//현재 drone이 위치한 gps값을 sub 
    ros::Subscriber _currentGohomeSpeedSubscriber;
    ros::Subscriber _currentGoEnteredCoordinatesSpeedSubscriber;
    ros::Subscriber _currentOdomSubscriber;
    ros::Subscriber _currentAttitudeSubscriber;
    
    sensor_msgs::NavSatFix                                  home_position;
    bebop_msgs::Ardrone3PilotingStatePositionChanged        current_position;
    bebop_msgs::Ardrone3PilotingStateAttitudeChanged        bebopAttitude;
    geometry_msgs::Twist                                    _bebopControlMessage;
    std_msgs::Bool                                          startNavigateHome;
    nav_msgs::Odometry                                      bebopOdom;
    
    bool _isTakeOff;
    bool _isGPS;
    bool _isGPSmode;
    bool _isGoHome;
    bool _isGoEnteredCoordinates;
    
    void _currentGPSCallback(const bebop_msgs::Ardrone3PilotingStatePositionChanged& currentGPS);
    void _currentGohomeSpeedCallback(const bebop_msgs::Ardrone3PilotingStatePositionChanged& currentGPS);
    void _currentGoEnteredCoordinatesCallback(const bebop_msgs::Ardrone3PilotingStatePositionChanged& currentGPS);
    void _currentOdomCallback(const nav_msgs::Odometry& currentOdom);
    void _currentAttitudeCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged& currentAttitude);
public:
    explicit BebopGlobalPositioningSystem(const ros::NodeHandle& nodeHandle);
};

#endif
