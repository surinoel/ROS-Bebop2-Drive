#ifndef BEBOP_TELEOP_BEBOP_TELEOP_H
#define BEBOP_TELEOP_BEBOP_TELEOP_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <termios.h>
#include <iostream>
#include <cstdlib>

#define UP 1
#define DOWN -1
#define FORWARD 1
#define BACKWARD -1
#define LEFT 1
#define RIGHT -1
#define CCW 1
#define CW -1

const bool trackOff = false;
const bool trackOn = true;
const bool gpsOff = false;
const bool gpsOn = true;
const bool gpsModeOff = false;
const bool gpsModeOn = true;
const bool takeoff = true;
const bool land = false;
const bool goHome = true;
const bool endGoHome = false;
const bool goEnteredcoordinates = true;
const bool endGoEnteredcoordinates = false;

/*파라미터 키 이름*/
const std::string gps_key = "/bebop/gps";
const std::string gps_mode_key = "/bebop/gpsmode";
const std::string takeoff_key = "/bebop/takeoff";
const std::string tracking_key = "/bebop/tracking";
const std::string home_gps_latitude_key = "/bebop/homegps/latitude";
const std::string home_gps_longitude_key = "/bebop/homegps/longitude";
const std::string drone_gps_latitude_key = "/bebop/dronegps/latitude";
const std::string drone_gps_longitude_key = "/bebop/dronegps/longitude";
const std::string go_home_key = "/bebop/gohome";
const std::string go_entered_coordinates_key = "/bebop/goenteredcoordinates";
const std::string go_latitude_key = "/bebop/go/latitude";
const std::string go_longitude_key = "/bebop/go/longitude";


class BebopKeyBoardController
{
private:
    ros::NodeHandle _nodeHandle;
    ros::Publisher _twistPublisher;
    ros::Publisher _takeOffPublisher;
    ros::Publisher _landPublisher;
    ros::Publisher _emergencyPublisher;

    geometry_msgs::Twist _controlValue;
    std_msgs::Empty _message;

    double _speedValue;
    double _speedIncreaseValue;
    bool _isTakeOff;
    bool _isTracking;
    bool _isGPS;
    bool _isGPSmode;
    
    static const char* Interface[];
    static const char* gpsInterface[];
    
    double home_gps_latitude;
    double home_gps_longitude;
    double drone_gps_latitude;
    double drone_gps_longitude;
    
    void _printInterface();
    void _printGpsInterface();
    void _move(double& value, int orientation);
    void _takeoff();
    void _land();
    void _emergency();
    void _speedUp();
    void _speedDown();
    char _getKey();

public:
    explicit BebopKeyBoardController(const ros::NodeHandle& nodeHandle);
    void Control();
};

#endif //BEBOP_TELEOP_BEBOP_TELEOP_H
