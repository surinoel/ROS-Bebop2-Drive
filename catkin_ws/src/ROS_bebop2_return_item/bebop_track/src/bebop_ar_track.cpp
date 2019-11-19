/*
  참고자료
  -quaternion 값에서 roll,pitch,yaw 값으로 변환하는 방법
  https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106
*/
#include <bebop_track/bebop_ar_track.h>
#include <math.h>
#include <ctime>
#include <string>
#include <iostream>
#include <algorithm>

#define LINEAR_SPEED            0.1//[meter/sec]
#define ANGULAR_SPEED           0.1//[radian/sec]
#define AR_MARKER_COUNT         50
#define CW                      -1//clock wise
#define CCW                     1//counter clock wise
#define RIGHT                   -1
#define LEFT                    1
#define STOP_DISTANCE           1.0

bool search_ar_marker = true;
bool find_ar_marker = false;
bool start_go_to_ar_marker = false;
bool is_callback_ar_marker_data = false;//ar marker data가 있을 때만 GoToArMarker()를 실행하기 위한 bool값
double dist_sum, ang_sum, dist_mean, sort_dist_mean, ang_mean, sort_ang_mean, dist_var, ang_var, dist_stdev, ang_stdev, y_dist, sort_y_dist;
double dist_data[AR_MARKER_COUNT];
double ang_data[AR_MARKER_COUNT];
int ar_marker_id = 0;

void PrintDistance(double* data)
{
    std::cout << "dist[i] (meter): ";
    for(int i = 0; i < AR_MARKER_COUNT; i++)
    {
        std::cout << data[i] << " ";
    }
    std::cout << std::endl;
}

void PrintAngle(double* data)
{
    std::cout << "angle[i] (degree): ";
    for(int i = 0; i < AR_MARKER_COUNT; i++)
    {
        std::cout << data[i]*(180.0/M_PI) << " ";
    }
    std::cout << std::endl;
}

double Sum(double data[], int count)
{
    double sum = 0.0;
    int i;

    for ( i = 0; i < count; ++i )
         sum += data[i];

    return sum;
}

double Mean(double data[], int count)
{
    return ( Sum(data, count) / count );
}

double SortMean(int start, int end, double data[])
{
    double sum = 0.0;
    int data_length = (end - start) + 1;
    int i;

    for( i = start; i <= end; ++i )
        sum += data[i];

    return (sum / data_length);
}

double Var(double data[], int count)
{
    int i;

    double sum = 0.0;
    double mean = Mean(data, count);

    for ( i = 0; i < count; ++i )
         sum += ( data[i] - mean ) * ( data[i] - mean );

    sum /= (double)(count - 1);

    return sum;
}

double Stdev(double data[], int count)
{
    return ( sqrt(Var(data, count)) );
}

void BebopArMarkerTrack::OdometryCallback(const nav_msgs::Odometry& odom)
{
    odometry_message.twist.twist.linear.y = odom.twist.twist.linear.y;
}

void BebopArMarkerTrack::ArMarkerPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& ar_marker)
{
    static bool get_once_ar_marker_data = true; // ar marker data를 한번만 받기위한 bool값
    static int count = 0;
    double roll, pitch, yaw;//armarker에서 반환해주는 quaternion값을 roll,pitch,yaw로 바꿔주면 pitch값을 이용해 bebop의 각도를 틀어준다.


    /*
        geometry_msgs/Quaternion.msg

        # This represents an orientation in free space in quaternion form.

        float64 x
        float64 y
        float64 z
        float64 w
    */
    geometry_msgs::Quaternion quaternion_msg;
    tf::Quaternion quaternion;

    alvar_marker_message = ar_marker;
    //취득한 ar marker 정보가 있을때만 밑에 for문이 돌아감
    for(int i=0; i<alvar_marker_message->markers.size(); i++) {

        ar_marker_id = 9;//추후 ar_marker_id_key 파라미터에 들어있는 값을 받도록 바꿀 예정
        // node_handle.getParam(ar_marker_id_key, ar_marker_id);// ar_marker_id_key에 들어있는 AR marker id 값을 ar_marker_id에 넣는다.
        if(ar_marker_id == alvar_marker_message->markers[i].id)
        {
            find_ar_marker = true;
            quaternion_msg.x = alvar_marker_message->markers[i].pose.pose.orientation.x;
            quaternion_msg.y = alvar_marker_message->markers[i].pose.pose.orientation.y;
            quaternion_msg.z = alvar_marker_message->markers[i].pose.pose.orientation.z;
            quaternion_msg.w = alvar_marker_message->markers[i].pose.pose.orientation.w;

            tf::quaternionMsgToTF(quaternion_msg, quaternion);//convert Quaternion msg to Quaternion
            tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);//Get the matrix represented as roll pitch and yaw about fixed axes XYZ.


            // if(get_once_ar_marker_data)
            // {
            //     dist_data[count] = alvar_marker_message->markers[i].pose.pose.position.z;//bebop2와 ar marker사이의 거리
            //     ang_data[count] = pitch;//bebop이랑 ar마커랑 yaw값 관계가 pitch(radian)로 나온다.
            //
            //     if(count >= (AR_MARKER_COUNT - 1))
            //     {
            //
            //         dist_sum = Sum(dist_data, AR_MARKER_COUNT);//거리 합
            //         dist_mean = Mean(dist_data, AR_MARKER_COUNT);//거리 평균
            //         dist_var = Var(dist_data, AR_MARKER_COUNT);//거리 분산
            //         dist_stdev = Stdev(dist_data, AR_MARKER_COUNT);//거리 표준편차
            //         ang_sum = Sum(ang_data, AR_MARKER_COUNT);//각도 합
            //         ang_mean = Mean(ang_data, AR_MARKER_COUNT);//각도 평균
            //         ang_var = Var(ang_data, AR_MARKER_COUNT);//각도 분산
            //         ang_stdev = Stdev(ang_data, AR_MARKER_COUNT);//각도 표준편차
            //         y_dist = dist_mean * sin(ang_mean);//y축 방향으로 이동 할 거리
            //
            //
            //
            //         PrintDistance(dist_data);//정렬 전
            //         PrintAngle(ang_data);
            //         std::cout << std::endl;
            //
            //         std::sort(dist_data, dist_data + AR_MARKER_COUNT);
            //         std::sort(ang_data, ang_data + AR_MARKER_COUNT);
            //         sort_dist_mean = SortMean(AR_MARKER_COUNT*2/5, AR_MARKER_COUNT*3/5, dist_data);//정렬한 값 중 2/5 ~ 3/5 사이에 있는 값들만 더해서 평균을 낸다.
            //         sort_ang_mean = SortMean(AR_MARKER_COUNT*2/5, AR_MARKER_COUNT*3/5, ang_data);
            //         sort_y_dist = sort_dist_mean * sin(sort_ang_mean);
            //
            //         PrintDistance(dist_data);//정렬 후
            //         PrintAngle(ang_data);
            //
            //         ROS_INFO("AR MARKER ID : %d", ar_marker_id);
            //         ROS_INFO("DIST SUM: %lf", dist_sum);
            //         ROS_INFO("DIST MEAN: %lf", dist_mean);
            //         ROS_INFO("SORT DIST MEAN: %lf", sort_dist_mean);
            //         ROS_INFO("DIST VAR: %lf", dist_var);
            //         ROS_INFO("DIST STDEV: %lf", dist_stdev);
            //         ROS_INFO("----------");
            //         ROS_INFO("ANG SUM: %lf", ang_sum*(180.0/M_PI));
            //         ROS_INFO("ANG MEAN: %lf", ang_mean*(180.0/M_PI));
            //         ROS_INFO("SORT ANG MEAN: %lf", sort_ang_mean*(180.0/M_PI));
            //         ROS_INFO("ANG VAR: %lf", ang_var*(180.0/M_PI));
            //         ROS_INFO("ANG STDEV: %lf", ang_stdev*(180.0/M_PI));
            //         ROS_INFO("----------");
            //         ROS_INFO("MOVE DISTANCE(y_dist): %lf", y_dist);
            //         ROS_INFO("SORT MOVE DISTANCE(y_dist): %lf", sort_y_dist);
            //         ROS_INFO("----------");
            //
            //         //set
            //         start_go_to_ar_marker = true;
            //         get_once_ar_marker_data = false;
            //     }
            //     ++count;
            // }

            is_callback_ar_marker_data = true;

            // ROS_INFO("DIST SUM: %lf", dist_sum);
            // ROS_INFO("DIST MEAN: %lf", dist_mean);
            // ROS_INFO("DIST VAR: %lf", dist_var);
            // ROS_INFO("DIST STDEV: %lf", dist_stdev);
            // ROS_INFO("----------");
            // ROS_INFO("ANG SUM: %lf", ang_sum*(180.0/M_PI));
            // ROS_INFO("ANG MEAN: %lf", ang_mean*(180.0/M_PI));
            // ROS_INFO("ANG VAR: %lf", ang_var*(180.0/M_PI));
            // ROS_INFO("ANG STDEV: %lf", ang_stdev*(180.0/M_PI));
            // ROS_INFO("----------");
            // ROS_INFO("MOVE DISTANCE(y_dist): %lf", y_dist);
            // ROS_INFO("----------");


            // ROS_INFO("Ar Marker id: %d", alvar_marker_message->markers[i].id);
            // ROS_INFO("Ar Marker position x: %lf", alvar_marker_message->markers[i].pose.pose.position.x);
            // ROS_INFO("Ar Marker position y: %lf", alvar_marker_message->markers[i].pose.pose.position.y);
            // ROS_INFO("Ar Marker position z: %lf", alvar_marker_message->markers[i].pose.pose.position.z);
            // ROS_INFO("Ar Marker orientation x: %lf", alvar_marker_message->markers[i].pose.pose.orientation.x);
            // ROS_INFO("Ar Marker orientation y: %lf", alvar_marker_message->markers[i].pose.pose.orientation.y);
            // ROS_INFO("Ar Marker orientation z: %lf", alvar_marker_message->markers[i].pose.pose.orientation.z);
            // ROS_INFO("Ar Marker orientation w: %lf", alvar_marker_message->markers[i].pose.pose.orientation.w);

            // ROS_INFO("Ar Marker id: %d", alvar_marker_message->markers[i].id);
            // ROS_INFO("current Ar Marker position z: %lf", alvar_marker_message->markers[i].pose.pose.position.z);
            // ROS_INFO("save Ar Marker position z: %lf", dist_mean);
            // ROS_INFO("MOVE DISTANCE(y_dist): %lf", y_dist);
            // ROS_INFO("----------");
            // ROS_INFO("current rpy angles(radian): pitch=%f", pitch);
            // ROS_INFO("current rpy angles(degree): pitch=%f", pitch*(180.0/M_PI));
            // ROS_INFO("save rpy angles(radian): pitch=%f", ang_mean);
            // ROS_INFO("save rpy angles(degree): pitch=%f", ang_mean*(180.0/M_PI));
            // ROS_INFO("----------");
            // ROS_INFO("Ar Marker id: %d", alvar_marker_message->markers[i].id);
            // ROS_INFO("current Ar Marker position x: %lf", alvar_marker_message->markers[i].pose.pose.position.x);
            // ROS_INFO("current Ar Marker position y: %lf", alvar_marker_message->markers[i].pose.pose.position.y);
            // ROS_INFO("current Ar Marker position z: %lf", alvar_marker_message->markers[i].pose.pose.position.z);
            // ROS_INFO("current rpy angles(degree): roll=%f", roll*(180.0/M_PI));
            // ROS_INFO("current rpy angles(degree): pitch=%f", pitch*(180.0/M_PI));
            // ROS_INFO("current rpy angles(degree): yaw=%f", yaw*(180.0/M_PI));
            // ROS_INFO("current quaternion angles(radian): x = %f", alvar_marker_message->markers[i].pose.pose.orientation.x);
            // ROS_INFO("current quaternion angles(radian): y = %f", alvar_marker_message->markers[i].pose.pose.orientation.y);
            // ROS_INFO("current quaternion angles(radian): z = %f", alvar_marker_message->markers[i].pose.pose.orientation.z);
            // ROS_INFO("current quaternion angles(radian): w = %f", alvar_marker_message->markers[i].pose.pose.orientation.w);
            // ROS_INFO("----------");
            continue;
        }
    }
    // find_ar_marker = false;
    // is_callback_ar_marker_data = false;
    // search_ar_marker = true;
}

void BebopArMarkerTrack::GoToArMarker()
{
    /*
        1.bebop이 ar marker를 인식하면
        2.bebop을 아래로 내려 높이를 맞추고
        3.bebop을 좌우로 움직여 ar marker를 일직선 상에서 바라보게 맞추고
        4.bebop을 앞으로 움직여 ar marker 앞으로 이동한다.
    */

    ros::Duration time(3.0);
    ros::Rate loop_rate(100);

    static bool first_step = true;
    static bool second_step = false;
    static bool third_step = false;
    static bool fourth_step = false;

    for(int i=0; i<alvar_marker_message->markers.size(); i++)
    {
        if(ar_marker_id == alvar_marker_message->markers[i].id)
        {
            if(first_step)
            {
                // if(fabs(alvar_marker_message->markers[0].pose.pose.position.y) > 0.02)
                // {
                //     if(alvar_marker_message->markers[0].pose.pose.position.y >= 0.0)//marker의 position.y가 +이면 bebop이 아래로 이동
                //     {
                //         bebop_control_message.linear.z = -LINEAR_SPEED;
                //         bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                //         ROS_INFO("GO DOWN!!!");
                //         ros::spinOnce();
                //         loop_rate.sleep();
                //         continue;
                //     }
                //     else if(alvar_marker_message->markers[0].pose.pose.position.y < 0.0)//marker의 position.y가 +이면 bebop이 아래로 이동
                //     {
                //         bebop_control_message.linear.z = LINEAR_SPEED;
                //         bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                //         ROS_INFO("GO DOWN!!!");
                //         ros::spinOnce();
                //         loop_rate.sleep();
                //         continue;
                //     }
                // }

                if(fabs(alvar_marker_message->markers[0].pose.pose.orientation.z) > 0.01)
                {
                    if(alvar_marker_message->markers[0].pose.pose.orientation.z < 0.0)//marker의 orientation.z가 -이면 오른쪽 회전
                    {
                        bebop_control_message.angular.z = -ANGULAR_SPEED/3;
                        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                        ROS_INFO("TURN RIGHT!!!");
                        ros::spinOnce();
                        loop_rate.sleep();
                        continue;
                    }
                    else if(alvar_marker_message->markers[0].pose.pose.orientation.z >= 0.0)//marker의 orientation.z가 +이면 왼쪽 회전
                    {
                        bebop_control_message.angular.z = ANGULAR_SPEED/3;
                        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                        ROS_INFO("TURN LEFT!!!");
                        ros::spinOnce();
                        loop_rate.sleep();
                        continue;
                    }
                }
                bebop_control_message.angular.z = 0;
                bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                time.sleep();
                first_step = false;
                second_step = true;
            }

            if(second_step)
            {
                if(fabs(alvar_marker_message->markers[0].pose.pose.position.x) > 0.01)//marker의 position.x가 -0.01 ~ 0.01 사이를 벗어나 있으면 동작
                {
                    if(alvar_marker_message->markers[0].pose.pose.position.x < 0.0)//marker의 position.x가 -이면 bebop이 왼쪽으로 이동
                    {
                        bebop_control_message.linear.y = LINEAR_SPEED/5;
                        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                        ROS_INFO("GO LEFT!!!");
                        ros::spinOnce();
                        loop_rate.sleep();
                        continue;
                    }
                    else if(alvar_marker_message->markers[0].pose.pose.position.x >= 0.0)//marker의 position.x가 +이면 bebop이 오른쪽으로 이동
                    {
                        bebop_control_message.linear.y = -LINEAR_SPEED/5;
                        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                        ROS_INFO("GO RIGHT!!!");
                        ros::spinOnce();
                        loop_rate.sleep();
                        continue;
                    }
                }
                bebop_control_message.linear.y = 0;
                bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                time.sleep();
                second_step = false;
                third_step = true;
            }

            if(third_step)
            {
                //position.z 0.5 = 50cm
                if(alvar_marker_message->markers[0].pose.pose.position.z >= 2.0)//marker의 position.z가 0.5보다 크면 bebop이 앞으로 이동
                {
                    bebop_control_message.linear.x = LINEAR_SPEED/5;
                    bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                    ROS_INFO("GO FRONT!!!");
                    ros::spinOnce();
                    loop_rate.sleep();
                    continue;
                }
                else
                {
                    bebop_control_message.linear.x = 0;
                    bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                    ROS_INFO("FINISH GO AR MARKER...");
                    // time.sleep();
                    // land_publisher.publish(empty_message);
                    for(int i = 0; i < 1000; ++i)
                    {
                        bebop_control_message.linear.z = LINEAR_SPEED*3;
                        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                    }
                    node_handle.setParam(go_ar_marker_key, end_go_ar_marker);
                    node_handle.setParam(go_home_key, go_home);
                }
                third_step = false;
            }
        }
    }

}

void BebopArMarkerTrack::GoToArMarker2()
{
    ros::Duration time(2);
    ros::Rate loop_rate(100);
    int direction_of_rotation = 0;
    int direction_of_movement = 0;
    static bool first_step = true;//ar마커의 정보를 여러번 읽어 평균값을 취한 값으로 ar마커가 보이는 것과 상관 없이 ar마커와 일직선을 대략 맞춤
    static bool second_step = false;//좌,우로 움직여 좀 더 미세하게 ar마커와 일직선을 맞춤
    static bool third_step = false;//ar마커와 일정거리가 될 때까지 bebop을 움직이고 landing
    double y_dist = sort_dist_mean * sin(sort_ang_mean);//bebop기준 y축 방향으로 움직일 거리
    double angle = sort_ang_mean;//bebop이 꺾어야 할 각도
    double time2turn = fabs(angle) / ANGULAR_SPEED;//회전하는데 걸리는 시간
    double time2go = fabs(y_dist) / LINEAR_SPEED;//bebop기준 y축 방향으로 움직이는데 걸리는 시간


    if(first_step)//ar마커의 정보를 여러번 읽어 평균값을 취한 값으로 ar마커가 보이는 것과 상관 없이 ar마커와 일직선을 대략 맞춤
    {

        if(angle >= 0.0 && alvar_marker_message->markers[0].pose.pose.position.x < 0.0)
        {
            ROS_INFO("ANGLE LARGER THEN 0.0");
            ROS_INFO("Bebop is on the right side of the ARmarker");
            direction_of_rotation = CW;
            direction_of_movement = LEFT;
        }
        else if(angle < 0 && alvar_marker_message->markers[0].pose.pose.position.x >= 0.0)
        {
            ROS_INFO("ANGLE LESS THEN 0.0");
            ROS_INFO("Bebop is on the left side of the ARmarker");
            direction_of_rotation = CCW;
            direction_of_movement = RIGHT;
        }
        // else if(angle >= 0 && alvar_marker_message->markers[0].pose.pose.position.x >= 0.0)
        // {
        //     ROS_INFO("ANGLE LARGER THEN 0.0");
        //     ROS_INFO("Bebop is on the left side of the ARmarker");
        //     direction_of_rotation = CW;
        //     direction_of_movement = RIGHT;
        // }
        // else if(angle < 0 && alvar_marker_message->markers[0].pose.pose.position.x < 0.0)
        // {
        //   ROS_INFO("ANGLE LESS THEN 0.0");
        //   ROS_INFO("Bebop is on the right side of the ARmarker");
        //   direction_of_rotation = CCW;
        //   direction_of_movement = LEFT;
        // }

        //회전
        ros::Time time2end = ros::Time::now() + ros::Duration(time2turn);//회전 종료 시간
        ros::Time start_time = ros::Time::now();//회전 시작 시간

        while(ros::Time::now() < time2end)//현재 시간이 종료 시간보다 작으면 while문을 실행한다.
        {
            ROS_INFO("SET ANGLE");
            ROS_INFO("start time: %lf", start_time.toSec());
            ROS_INFO("end time: %lf", time2end.toSec());
            ROS_INFO("current time: %lf", ros::Time::now().toSec());
            ROS_INFO("----------");
            bebop_control_message.angular.z = direction_of_rotation * ANGULAR_SPEED;
            bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
            loop_rate.sleep();
        }
        bebop_control_message.angular.z = 0.0;
        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);

        //이동
        time2end = ros::Time::now() + ros::Duration(time2go);//이동 종료 시간
        start_time = ros::Time::now();//이동 시작 시간

        while(ros::Time::now() < time2end)//현재 시간이 종료 시간보다 작으면 while문을 실행한다.
        {
            ROS_INFO("SET DIRECTION");
            ROS_INFO("start time: %lf", start_time.toSec());
            ROS_INFO("end time: %lf", time2end.toSec());
            ROS_INFO("current time: %lf", ros::Time::now().toSec());
            ROS_INFO("----------");

            double current_time2go = fabs(y_dist) / fabs(odometry_message.twist.twist.linear.y);
            if(fabs(odometry_message.twist.twist.linear.y) < LINEAR_SPEED || current_time2go > time2go)
            {
                bebop_control_message.linear.y = direction_of_movement * LINEAR_SPEED;
                bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
        }

        bebop_control_message.linear.y = 0;
        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
        time.sleep();
        ROS_INFO("start time: %lf", start_time.toSec());
        ROS_INFO("end time: %lf", time2end.toSec());
        ROS_INFO("DIST SUM: %lf", dist_sum);
        ROS_INFO("DIST MEAN: %lf", dist_mean);
        ROS_INFO("SORT DIST MEAN: %lf", sort_dist_mean);
        ROS_INFO("DIST VAR: %lf", dist_var);
        ROS_INFO("DIST STDEV: %lf", dist_stdev);
        ROS_INFO("----------");
        ROS_INFO("ANG SUM: %lf", ang_sum*(180.0/M_PI));
        ROS_INFO("ANG MEAN: %lf", ang_mean*(180.0/M_PI));
        ROS_INFO("SORT ANG MEAN: %lf", sort_ang_mean*(180.0/M_PI));
        ROS_INFO("ANG VAR: %lf", ang_var*(180.0/M_PI));
        ROS_INFO("ANG STDEV: %lf", ang_stdev*(180.0/M_PI));
        ROS_INFO("----------");
        ROS_INFO("MOVE DISTANCE(y_dist): %lf", y_dist);
        ROS_INFO("SORT MOVE DISTANCE(sort_y_dist): %lf", sort_y_dist);
        ROS_INFO("----------");
        first_step = false;//첫번째 단계를 종료하고
        second_step = true;//두번째 단계로 넘어간다
        //start_go_to_ar_marker = false;
    }

    if(second_step)//좌,우로 움직여 좀 더 미세하게 ar마커와 일직선을 맞춤(오차범위 -2cm ~ 2cm)
    {
        for(int i=0; i<alvar_marker_message->markers.size(); i++) {
            if(i==0)
            {
                if(fabs(alvar_marker_message->markers[0].pose.pose.position.x) > 0.02)//marker의 position.x가 -0.02 ~ 0.02(-2cm ~ 2cm) 사이를 벗어나 있으면 동작
                {
                    if(alvar_marker_message->markers[0].pose.pose.position.x < 0.0)//marker의 position.x가 -이면 bebop이 왼쪽으로 이동
                    {
                        bebop_control_message.linear.y = LINEAR_SPEED/3;
                        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        // loop_rate.sleep();
                        ROS_INFO("GO LEFT!!!");
                        continue;
                    }
                    else if(alvar_marker_message->markers[0].pose.pose.position.x >= 0.0)//marker의 position.x가 +이면 bebop이 오른쪽으로 이동
                    {
                        bebop_control_message.linear.y = -LINEAR_SPEED/3;
                        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        // loop_rate.sleep();
                        ROS_INFO("GO RIGHT!!!");
                        continue;
                    }
                }
            }
            bebop_control_message.linear.y = 0.0;
            bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
            time.sleep();
            //marker와 bebop의 position.x 관계가 -0.02 ~ 0.02(-2cm ~ 2cm) 사이라면 두번째 단계를 종료하고 세번째 단계 시작
            second_step = false;
            third_step = true;
        }
    }

    if(third_step)//ar마커와 일정거리가 될 때까지 bebop을 움직이고 landing
    {
        for(int i=0; i<alvar_marker_message->markers.size(); i++) {
            if(i==0)
            {
                if(alvar_marker_message->markers[0].pose.pose.position.z >= STOP_DISTANCE)//marker의 position.z가 1.5[meter]보다 크면 bebop이 앞으로 이동
                {
                    bebop_control_message.linear.x = LINEAR_SPEED/3;
                    bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
                    loop_rate.sleep();
                    ROS_INFO("GO FRONT!!!");
                    continue;
                }
            }
            bebop_control_message.linear.x = 0.0;
            bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
            ROS_INFO("FINISH GO AR MARKER...");
            time.sleep();
            land_publisher.publish(empty_message);
            node_handle.setParam(go_ar_marker_key, end_go_ar_marker);
            node_handle.setParam(go_home_key, go_home);
            third_step = false;
            start_go_to_ar_marker = false;
        }
    }
}

void BebopArMarkerTrack::SearchArMarker()
{
    ros::Duration time(3.0);
    ros::Rate loop_rate(50);
    find_ar_marker = false;
    while(ros::ok())
    {
        ROS_INFO("search AR marker...");
        bebop_control_message.angular.z =  ANGULAR_SPEED;
        bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
        ros::spinOnce();
        loop_rate.sleep();

        if(find_ar_marker)
        {
          ROS_INFO("find AR marker...");
          bebop_control_message.angular.z = 0;
          bebop_control_from_ar_marker_publisher.publish(bebop_control_message);
          time.sleep();
          search_ar_marker = false;
          break;
        }
    }

}

void BebopArMarkerTrack::GetParam()
{
    node_handle.getParam(go_ar_marker_key, does_go_ar_marker);
}

void BebopArMarkerTrack::Action()
{
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        GetParam();

        //does_go_ar_marker는 bebop이 gps위치에 도착하면 true가 된다.
        //gps위치에 도착하면 ar마커를 찾기 시작한다.
        //ar마커를 찾으면 search_ar_marker가 false가 된다.
        if(search_ar_marker /*&& does_go_ar_marker*/)
        {
            SearchArMarker();
        }
        //
        if(is_callback_ar_marker_data /*&& start_go_to_ar_marker*/ /*&& does_go_ar_marker*/)
        {
            GoToArMarker();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}



//생성자
BebopArMarkerTrack::BebopArMarkerTrack(const ros::NodeHandle& nh)
:node_handle(nh),
land_publisher(node_handle.advertise<std_msgs::Empty>("bebop/land", 1)),
ar_marker_pose_subscriber(node_handle.subscribe(ar_marker_pose, 1, &BebopArMarkerTrack::ArMarkerPoseCallback, this)),
current_odometry_subscriber(node_handle.subscribe(bebop_odom, 1, &BebopArMarkerTrack::OdometryCallback, this)),
bebop_control_from_ar_marker_publisher(node_handle.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10))
{

}
