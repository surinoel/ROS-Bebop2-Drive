#ifndef __BEBOP_GPS_H__
#define __BEBOP_GPS_H__

#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include "bebop_msgs/Ardrone3PilotingStatePositionChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"

//파라미터를 set할때 사용하는 변수
const bool gps_off = false;
const bool gps_on = true;
const bool gps_mode_off = false;
const bool gps_mode_on = true;
const bool go_home = true;
const bool end_go_home = false;
const bool go_entered_coordinates = true;
const bool end_go_entered_coordinates = false;
const bool go_place = true;
const bool end_go_place = false;
const bool go_ar_marker = true;
const bool end_go_ar_marker = false;

//파라미터 키 이름
const std::string gps_key = "/bebop/gps";
const std::string gps_mode_key = "/bebop/gpsmode";
const std::string take_off_key = "/bebop/takeoff";
const std::string home_gps_latitude_key = "/bebop/homegps/latitude";
const std::string home_gps_longitude_key = "/bebop/homegps/longitude";
const std::string drone_gps_latitude_key = "/bebop/dronegps/latitude"; // teleop 화면에 gps latitude값을 띄우기 위한 param key
const std::string drone_gps_longitude_key = "/bebop/dronegps/longitude"; // teleop 화면에 gps longitude값을 띄우기 위한 param key
const std::string drone_gps_altitude_key = "/bebop/dronegps/altitude";
const std::string go_home_key = "/bebop/gohome";
const std::string go_entered_coordinates_key = "/bebop/goenteredcoordinates";
const std::string go_latitude_key = "/bebop/go/latitude";
const std::string go_longitude_key = "/bebop/go/longitude";
const std::string select_place_key = "/bebop/select/place";
const std::string go_place_key = "/bebop/go/place";
const std::string go_ar_marker_key = "/bebop/go/armarker";
const std::string ar_marker_id_key = "/bebop/armarker/id";


//subscribe하는 topic 이름
const std::string bebop_position_changed = "/bebop/states/ardrone3/PilotingState/PositionChanged";//드론의 gps값을 얻기위한 topic
const std::string bebop_attitude_changed = "/bebop/states/ardrone3/PilotingState/AttitudeChanged";//드론의 yaw값을 얻기위한 topic

class BebopGlobalPositioningSystem
{
private:
    ros::NodeHandle node_handle;

    ros::Publisher bebop_control_from_gps_publisher;//목표 위치와 현재 위치를 비교해서 x방향 선속도와 각속도를 pub
    ros::Publisher bebop_go_home_publisher;//home 방향으로 기체 머리를 돌리고 고도를 맞추는 pub
    ros::Publisher land_publisher;//land message를 publish하는 publisher
    ros::Subscriber current_gps_subscriber;//현재 drone이 위치한 gps값을 sub
    ros::Subscriber current_attitude_subscriber;//현재 drone의 yaw값을 sub

    sensor_msgs::NavSatFix                                  home_position;//home으로 지정한 위치의 위도(latitude), 경도(longitude)값을 담을 변수
    bebop_msgs::Ardrone3PilotingStatePositionChanged        current_position;//드론의 현재 위도(latitude), 경도(longitude), 고도(altitude)값을 담을 변수
    bebop_msgs::Ardrone3PilotingStateAttitudeChanged        bebop_attitude;//드론의 yaw값을 담을 변수
    geometry_msgs::Twist                                    bebop_control_message;//선속도,각속도를 담아 드론을 control할 변수
    std_msgs::Bool                                          start_navigate_home;//true이면 home방향으로 드론 머리를 돌리고 고도를 일정 고도(bebop 내부에 정해져 있는듯하다.)로 맞춘다.
    std_msgs::Empty                                         empty_message;//land시키기 위한 변수

    bool does_take_off;                //is take off?                 yes:true, no:false
    bool is_gps;                    //is gps?                     yes:true, no:false
    bool does_go_home;                 //is go home?                 yes:true, no:false
    bool does_go_entered_coordinates;   //is go entered coordinates?  yes:true, no:false
    bool does_go_place;                //is go place?                yes:true, no:false

    //Subscriber Call back function
    void CurrentGpsCallback(const bebop_msgs::Ardrone3PilotingStatePositionChanged& current_gps);//현재 gps값을 call back
    void CurrentAttitudeCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged& current_attitude);//현재 drone의 yaw값을 call back

    void GetParam();//파라미터들의 상태를 얻어옴
    void ReturnHome();//집으로 설정된 위도(latitude),경도(longitude)로 가는 함수
    void ControlGps();
    void GoEnteredCoordinates();//입력한 위도,경도 값으로 가는 함수
    void GoSelectedPlace();//선택된 장소의 위도,경도 값으로 가는 함수
public:

    void Action();//파라미터 값에 따라 drone의 행동을 결정하는 함수
    explicit BebopGlobalPositioningSystem(const ros::NodeHandle& nh);//생성자
};

#endif
