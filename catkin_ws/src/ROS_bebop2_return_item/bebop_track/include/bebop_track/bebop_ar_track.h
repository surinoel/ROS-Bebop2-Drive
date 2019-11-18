#ifndef __BEBOP_AR_TRACK_H__
#define __BEBOP_AR_TRACK_H__
/*
    #사용한 AR 마커 한 변의 길이 = 18.5cm
    #AR마커는 position 값을 meter단위로 publish 해준다.
    #AR마커에서 publish 해주는 position z 값을 cm로 환산해보면
     0.01 = 1cm
     ex) 0.949044 = 대략 94cm
     이 거리는 AR마커와 bebop2 drone 카메라 사이의 거리이다.
*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
//#include <LinearMath/btMatrix3x3.h>


//파라미터를 set할때 사용하는 변수들
const bool go_home = true;
const bool end_go_ar_marker = false;

//파라미터 키 이름
const std::string go_home_key = "/bebop/gohome";
const std::string go_ar_marker_key = "/bebop/go/armarker";
const std::string ar_marker_id_key = "/bebop/armarker/id";

//subscribe하는 topic 이름
const std::string ar_marker_pose = "/ar_pose_marker";//ar marker의 pose값을 subscribe 하기 위한 topic
const std::string bebop_odom = "/bebop/odom";//bebop의 odometry값을 subscribe 하기 위한 topic

class BebopArMarkerTrack
{
private:
    ros::NodeHandle node_handle;

    ros::Publisher land_publisher;//land message를 publish하는 publisher
    ros::Publisher bebop_control_from_ar_marker_publisher;//ar marker pose(position 값과 orientation 값)를 기준으로 bebop을 control
    ros::Subscriber ar_marker_pose_subscriber;//ar marker의 pose 값을 sub
    ros::Subscriber current_attitude_subscriber;//현재 drone의 yaw값을 sub
    ros::Subscriber current_odometry_subscriber;//현재 drone의 odometry값을 sub

    bool does_go_ar_marker;

    geometry_msgs::Twist                                    bebop_control_message;//선속도,각속도를 담아 드론을 control할 변수
    nav_msgs::Odometry                                      odometry_message;//bebop의 odometry값을 담을 변수
    ar_track_alvar_msgs::AlvarMarkers::ConstPtr             alvar_marker_message;//marker의 pose값을 담을 변수
    geometry_msgs::Point                                    ar_marker_point_message;//ar marker의 x,y,z값을 담을 변수
    geometry_msgs::Vector3                                  ar_marker_rpy_message;//quaternion값을 roll,pitch,yaw 값으로 변형한 값을 저장할 변수
    std_msgs::Empty                                         empty_message;//land시키기 위한 변수

    //Subscriber Call back function
    void ArMarkerPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& ar_marker);
    void OdometryCallback(const nav_msgs::Odometry& odom);

    //function
    void GetParam();
    void SearchArMarker();
    void GoToArMarker();//마커의 포즈값에 따라 bebop2를 조종해 마커 앞으로 가는 함수
    void GoToArMarker2();
public:
    void Action();
    explicit BebopArMarkerTrack(const ros::NodeHandle& nh);//생성자
};

#endif
