#include <bebop_track/bebop_gps.h>
#include <bebop_track/gps_to_xy.h>
#include <bebop_track/print.h>

#define UP_ALTITUDE         3.0
#define DOWN_ALTITUDE       1.0
#define ANGULAR_SPEED       0.3
#define LINEAR_SPEED        0.2
#define DOWN_LINEAR_SPEED   0.02
#define DOWN_ANGULAR_SPEED  0.01
#define DOWN_ANG_SPEED_GAP  0.2
#define DOWN_SPEED_DISTANCE 4.0
#define STOP_DISTANCE       3.0
#define dt                  0.001
#define TOLERANCE           1
#define TOLERANCEYAW        3
#define maxSpeedX           0.3
#define maxSpeedY           0.3
#define maxSpeedZ           0.3
#define maxSpeedYaw         0.1
/*
    bebop 2

    Max horizontal speed: 16 m/s
    Max upward speed: 6 m/s
    Max rotation speed: 200 degree/s
    Max tilt speed: 300 degree/s
    Max tilt: 35 degree
*/


bool ready_to_get_home_gps = true;
bool start_altitude_set = true;
bool end_altitude_set = true;
bool start_navigate_home_set = true;
bool target_angle_set = true;
bool go_target_gps_set = true;

int num = 0;

////////////////////////////////////////////
void BebopGlobalPositioningSystem::CurrentGpsCallback(const bebop_msgs::Ardrone3PilotingStatePositionChanged& current_gps)
{
    ros::Rate loop_rate(100);

    node_handle.getParam(take_off_key, does_take_off);
    node_handle.getParam(gps_key, is_gps);

    if(does_take_off && is_gps)
    {
        if(ready_to_get_home_gps) // 처음 이륙한 위치를 home위치로 설정
        {
            home_position.latitude = current_gps.latitude;
            home_position.longitude = current_gps.longitude;
            node_handle.setParam(home_gps_latitude_key, home_position.latitude); //사용자 인터페이스 화면에 표시해주기 위해서
            node_handle.setParam(home_gps_longitude_key, home_position.longitude);//사용자 인터페이스 화면에 표시해주기 위해서

            ready_to_get_home_gps = false; //home_position 값을 다시 받지 않기 위해 ready_to_get_home_gps를 false로 변경
        }

        current_position.latitude  = current_gps.latitude;//위도
        current_position.longitude = current_gps.longitude;//경도
        current_position.altitude = current_gps.altitude;//고도
        node_handle.setParam(drone_gps_latitude_key, current_position.latitude);
        node_handle.setParam(drone_gps_longitude_key, current_position.longitude);
        node_handle.setParam(drone_gps_altitude_key, current_position.altitude);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
///////////////////////////////////
void BebopGlobalPositioningSystem::CurrentAttitudeCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged& current_attitude)
{
    bebop_attitude.yaw = current_attitude.yaw;//드론이 북쪽을 바라볼 때 yaw값이 0이 나온다. 드론이 북쪽을 바라볼 때 왼쪽으로 -0.0 ~ -3.141592 [radian], 오른쪽으로 0.0 ~ 3.141592 [radian]의 값을 가진다.
}
/////////////////////////////////
void BebopGlobalPositioningSystem::ReturnHome()
{
    ros::Duration time(3);
    ros::Rate loop_rate(50);

    while(does_go_home) // h/H 키가 눌렸으면
    {
        // if(start_altitude_set)
        // {
        //     if(current_position.altitude < UP_ALTITUDE)
        //     {
        //         bebop_control_message.linear.z = LINEAR_SPEED;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //         continue;
        //     }
        //     else
        //     {
        //         bebop_control_message.linear.z = 0.0;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);
        //         start_altitude_set = false;
        //     }
        // }

        if(start_navigate_home_set)
        {
            time.sleep();
            start_navigate_home.data = true;
            bebop_go_home_publisher.publish(start_navigate_home); // /bebop/autoflight/navigate_home 토픽에 true를 pub, home gps좌표로 머리를 틀고 고도를 맞춘다.

            start_navigate_home_set = false;
        }

        ros::spinOnce();//?
        loop_rate.sleep();//?

        //현재 gps와 목표 gps 사이의 거리가 STOP_DISTANCE보다 크면 bebop2의 linear.x에 LINEAR_SPEED를 넣어주고 pub해준다.
        if(calDistance(home_position.latitude, home_position.longitude, current_position.latitude, current_position.longitude) > STOP_DISTANCE)
        {
            bebop_control_message.linear.x = LINEAR_SPEED;
            bebop_control_message.linear.y = 0;
            bebop_control_message.angular.x = 0;
            bebop_control_message.angular.y = 0;
            bebop_control_message.angular.z = 0;
            bebop_control_from_gps_publisher.publish(bebop_control_message);

            ros::spinOnce();//spinOnce를 하면 자신을 포함한 모든 콜백함수가 호출된다.
            loop_rate.sleep();

            continue;//
        }

        bebop_control_message.linear.x = 0.0;
        bebop_control_from_gps_publisher.publish(bebop_control_message);

        // if(end_altitude_set)
        // {
        //     if(current_position.altitude > DOWN_ALTITUDE)
        //     {
        //         bebop_control_message.linear.z = -LINEAR_SPEED;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);

        //         ros::spinOnce();
        //         loop_rate.sleep();

        //         continue;
        //     }
        //     else
        //     {
        //         bebop_control_message.linear.z = 0.0;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);
        //         time.sleep();

        //         land_publisher.publish(empty_message);

        //         end_altitude_set = false;
        //     }
        // }

        break;
        //node_handle.setParam(gps_key, gps_off);
    }
    start_altitude_set = true;
    start_navigate_home_set = true;
    end_altitude_set = true;
    node_handle.setParam(go_home_key, end_go_home);
}
//////
void BebopGlobalPositioningSystem::ControlGps(){
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

    double go_latitude = 0.0;
    double go_longitude = 0.0;
    double target_angle = 0.0;

    node_handle.getParam(go_latitude_key, go_latitude);
    node_handle.getParam(go_longitude_key, go_longitude);

  	ros::Rate loopRate(50);

    while(ros::ok())
    {
        // if(start_altitude_set)
        // {
        //     if(current_position.altitude < UP_ALTITUDE)
        //     {
        //         bebop_control_message.linear.z = LINEAR_SPEED;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);

        //         ros::spinOnce();
        //         loopRate.sleep();

        //         continue;
        //     }
        //     else
        //     {
        //       bebop_control_message.linear.z = 0.0;
        //       bebop_control_from_gps_publisher.publish(bebop_control_message);

        //       start_altitude_set = false;
        //     }
        // }

      	if(calDistance(go_latitude, go_longitude, current_position.latitude, current_position.longitude) >= TOLERANCE || abs(radian_to_degree(yaw_gap)) >= TOLERANCEYAW){	// distance from drone' position to destination

        		longitude_gap = go_longitude - current_position.longitude;
        		latitude_gap = go_latitude - current_position.latitude;
        		//altitude_gap = home_position.altitude - current_position.altitude;

        		if(bebop_attitude.yaw >= -1.5 && bebop_attitude.yaw <= 3.)
        			theta = 1.5 - bebop_attitude.yaw;	// radian between x axis and drone' x vector
        		else
        			theta = -4.5 - bebop_attitude.yaw;

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

        		printHomeGPS(home_position.longitude, home_position.latitude, home_position.altitude);
        		printDroneGPS(current_position.longitude, current_position.latitude, current_position.altitude);
        		printRotation(bebop_attitude.roll, bebop_attitude.pitch, bebop_attitude.yaw);
        		printGap(calDistance(current_position.latitude, current_position.longitude, home_position.latitude, home_position.longitude), longitude_gap, latitude_gap, altitude_gap, yaw_gap);
        		ROS_INFO("[GAP] YAWGAP : %lf", radian_to_degree(yaw_gap));

        		// move to destination
        		bebop_control_message.linear.x = speedX / dt;	//pX + iX + dX;
        		if (bebop_control_message.linear.x > maxSpeedX) bebop_control_message.linear.x = maxSpeedX;				// define the max speed of x
        		else if (bebop_control_message.linear.x < -maxSpeedX) bebop_control_message.linear.x = -maxSpeedX;
        		bebop_control_message.linear.y = speedY / dt;	// pY + iY + dY;
        		if (bebop_control_message.linear.y > maxSpeedY) bebop_control_message.linear.y = maxSpeedY;				// define the max speed of y
        		else if (bebop_control_message.linear.y < -maxSpeedY) bebop_control_message.linear.y = -maxSpeedY;
        		// bebop_control_message.linear.z = altitude_gap * dt;	// Z + iZ + dZ;
        		// if (bebop_control_message.linear.z > maxSpeedZ) bebop_control_message.linear.z = maxSpeedZ;				// define the max speed of z
        		// else if (bebop_control_message.linear.z < -maxSpeedZ) bebop_control_message.linear.z = -maxSpeedZ;
        		bebop_control_message.angular.z = speedYaw;	//Yaw + iYaw + dYaw;
        		if (bebop_control_message.angular.z > maxSpeedYaw) bebop_control_message.angular.z = maxSpeedYaw;	// define the max speed of yaw
        		else if (bebop_control_message.angular.z < -maxSpeedYaw) bebop_control_message.angular.z = -maxSpeedYaw;

        		bebop_control_message.angular.x = bebop_control_message.angular.y = 0;

        		printSpeed(bebop_control_message.linear.x, bebop_control_message.linear.y, bebop_control_message.linear.z, bebop_control_message.angular.z);

        		bebop_control_from_gps_publisher.publish(bebop_control_message);
        		ros::spinOnce();	// subscribe all sub data
        		loopRate.sleep();
            continue;
      	}

        // if(end_altitude_set)
        // {
        //     if(current_position.altitude > DOWN_ALTITUDE)
        //     {
        //         bebop_control_message.linear.z = -LINEAR_SPEED;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);

        //         ros::spinOnce();
        //         loopRate.sleep();

        //         continue;
        //     }
        //     else
        //     {
        //         bebop_control_message.linear.z = 0.0;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);

        //         end_altitude_set = false;
        //     }
        // }
        break;
    }
        start_altitude_set = true;
        end_altitude_set = true;
        node_handle.setParam(go_entered_coordinates_key, end_go_entered_coordinates);
        node_handle.setParam(go_place_key, end_go_place);
        node_handle.setParam(go_ar_marker_key, go_ar_marker);
}

////////////////////////////
void BebopGlobalPositioningSystem::GoEnteredCoordinates()
{
    ros::Rate loop_rate(50);

    /*double test_target_angle = bearingP1toP2(current_position.latitude, current_position.longitude, home_position.latitude, home_position.longitude);//현재 gps와 home gps 사이의 각도(테스트 용도)
    double test_distance = calDistance(home_position.latitude, home_position.longitude, current_position.latitude, current_position.longitude);//현재 gps와 home gps 사이의 거리(테스트 용도)*/

    //std::cout << "Target angle:" << test_target_angle << std::endl;
    //std::cout << "Bebop  angle:" << bebop_attitude.yaw << std::endl;
    //std::cout << "Test distance:" << test_distance << std::endl;
    //std::cout << "does_go_entered_coordinates state:" << does_go_entered_coordinates << std::endl;
    double go_latitude = 0.0;
    double go_longitude = 0.0;
    double target_angle = 0.0;

    node_handle.getParam(go_latitude_key, go_latitude);
    node_handle.getParam(go_longitude_key, go_longitude);
    target_angle = bearingP1toP2(current_position.latitude, current_position.longitude, go_latitude, go_longitude);

    while(ros::ok())
    {
        //std::cout << "Entered go coordinates..." << std::endl;
        //const bebop_msgs::Ardrone3PilotingStateAttitudeChanged current_attitude;

        //ROS_INFO("target_angle : %lf", target_angle);

        // if(start_altitude_set)
        // {
        //     if(current_position.altitude < UP_ALTITUDE)
        //     {
        //         bebop_control_message.linear.z = LINEAR_SPEED;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //         continue;
        //     }
        //     else
        //     {
        //       bebop_control_message.linear.z = 0.0;
        //       bebop_control_from_gps_publisher.publish(bebop_control_message);
        //       start_altitude_set = false;
        //     }
        // }
        ///////
        if(target_angle_set)
        {
            //std::cout << "--- target angle set ---" << std::endl;
            //사분면에 따라 다르게 회전
            if((go_latitude < current_position.latitude) && (go_longitude < current_position.longitude)) // 목표위치 기준 1사분면이면
            {
                if(0.0 < bebop_attitude.yaw && bebop_attitude.yaw <= (1.0/2.0)*M_PI) // yaw state 1
                {
                    //std::cout << "Quadrant 1" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;


                    if(target_angle - fabs(bebop_attitude.yaw) < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = DOWN_ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < fabs(bebop_attitude.yaw))
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = (target_angle - fabs(bebop_attitude.yaw))/1.5; // 왼쪽으로 회전
                    bebop_control_message.angular.z = ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if(-(1.0/2.0)*M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= -0.0) // yaw state 2
                {
                    //std::cout << "Quadrant 1" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(target_angle - fabs(bebop_attitude.yaw) < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = DOWN_ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < fabs(bebop_attitude.yaw))
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = (target_angle - fabs(bebop_attitude.yaw))/1.5; // 왼쪽으로 회전
                    bebop_control_message.angular.z = ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if(-M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= -(1.0/2.0)*M_PI) // yaw state 3
                {
                    //std::cout << "Quadrant 1" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(fabs(bebop_attitude.yaw) < target_angle)
                    {
                        if(target_angle - fabs(bebop_attitude.yaw) < DOWN_ANG_SPEED_GAP)
                        {
                            bebop_control_message.angular.z = DOWN_ANGULAR_SPEED;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);
                            ros::spinOnce();
                            loop_rate.sleep();

                            if(target_angle < fabs(bebop_attitude.yaw))
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                bebop_control_message.angular.z = 0.0;
                                bebop_control_from_gps_publisher.publish(bebop_control_message);

                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //bebop_control_message.angular.z = (target_angle - fabs(bebop_attitude.yaw))/1.5; // 왼쪽으로 회전
                        bebop_control_message.angular.z = ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    else if(fabs(bebop_attitude.yaw) >= target_angle)
                    {
                        if(fabs(bebop_attitude.yaw) - target_angle < DOWN_ANG_SPEED_GAP)
                        {
                            bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                            bebop_control_from_gps_publisher.publish(bebop_control_message);
                            ros::spinOnce();
                            loop_rate.sleep();

                            if(fabs(bebop_attitude.yaw) < target_angle) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                bebop_control_message.angular.z = 0.0;
                                bebop_control_from_gps_publisher.publish(bebop_control_message);

                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //bebop_control_message.angular.z = -(fabs(bebop_attitude.yaw) - target_angle)/1.5; // 오른쪽으로 회전
                        bebop_control_message.angular.z = -ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                else if((1.0/2.0)*M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= M_PI) // yaw state 4
                {
                    //std::cout << "Quadrant 1" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(target_angle + bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle > bebop_attitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = -(target_angle + bebop_attitude.yaw)/1.5; // 오른쪽으로 회전
                    bebop_control_message.angular.z = -ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                continue;
            }//Quadrant 1 end
            else if((go_latitude < current_position.latitude) && (go_longitude > current_position.longitude))//목표위치 기준 2사분면이면
            {
                if(0.0 < bebop_attitude.yaw && bebop_attitude.yaw <= (1.0/2.0)*M_PI) // yaw state 1
                {
                    //std::cout << "Quadrant 2" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;


                    if(target_angle - bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < bebop_attitude.yaw)
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = -(target_angle - bebop_attitude.yaw)/1.5; // 오른쪽으로 회전
                    bebop_control_message.angular.z = -ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();

                }
                else if(-(1.0/2.0)*M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= -0.0) // yaw state 2
                {
                    //std::cout << "Quadrant 2" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(target_angle - bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < bebop_attitude.yaw)
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = -(target_angle - bebop_attitude.yaw)/1.5; // 오른쪽으로 회전
                    bebop_control_message.angular.z = -ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if(-M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= -(1.0/2.0)*M_PI) // yaw state 3
                {
                    //std::cout << "Quadrant 2" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;


                    if(target_angle - bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < bebop_attitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = -(target_angle - bebop_attitude.yaw)/1.5; // 오른쪽으로 회전
                    bebop_control_message.angular.z = -ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();

                }
                else if((1.0/2.0)*M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= M_PI) // yaw state 4
                {
                    //std::cout << "Quadrant 2" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(bebop_attitude.yaw > target_angle)
                    {
                        if(bebop_attitude.yaw - target_angle < DOWN_ANG_SPEED_GAP)
                        {
                            bebop_control_message.angular.z = DOWN_ANGULAR_SPEED; // 왼쪽 회전
                            bebop_control_from_gps_publisher.publish(bebop_control_message);
                            ros::spinOnce();
                            loop_rate.sleep();

                            if(bebop_attitude.yaw < target_angle) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                bebop_control_message.angular.z = 0.0;
                                bebop_control_from_gps_publisher.publish(bebop_control_message);

                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //bebop_control_message.angular.z = (bebop_attitude.yaw - target_angle)/1.5; // 왼쪽으로 회전
                        bebop_control_message.angular.z = ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }

                    else if(bebop_attitude.yaw <= target_angle)
                    {
                        if(target_angle - bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                        {
                            bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                            bebop_control_from_gps_publisher.publish(bebop_control_message);
                            ros::spinOnce();
                            loop_rate.sleep();

                            if(target_angle < bebop_attitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                bebop_control_message.angular.z = 0.0;
                                bebop_control_from_gps_publisher.publish(bebop_control_message);

                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //bebop_control_message.angular.z = -(target_angle - bebop_attitude.yaw)/1.5; // 오른쪽으로 회전
                        bebop_control_message.angular.z = -ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                continue;
            }//Quadrant 2 end
            else if((go_latitude > current_position.latitude) && (go_longitude > current_position.longitude))//목표위치 기준 3사분면이면
            {
                if(0.0 < bebop_attitude.yaw && bebop_attitude.yaw <= (1.0/2.0)*M_PI) // yaw state 1
                {
                    //std::cout << "Quadrant 3" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;


                    if(bebop_attitude.yaw < target_angle)
                    {
                        if(target_angle - bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                        {
                            bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                            bebop_control_from_gps_publisher.publish(bebop_control_message);
                            ros::spinOnce();
                            loop_rate.sleep();

                            if(target_angle < bebop_attitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                bebop_control_message.angular.z = 0.0;
                                bebop_control_from_gps_publisher.publish(bebop_control_message);

                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //bebop_control_message.angular.z = -(target_angle - bebop_attitude.yaw)/1.5; // 오른쪽으로 회전
                        bebop_control_message.angular.z = -ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    else if(bebop_attitude.yaw >= target_angle)
                    {
                        if(bebop_attitude.yaw - target_angle < DOWN_ANG_SPEED_GAP)
                        {
                            bebop_control_message.angular.z = DOWN_ANGULAR_SPEED;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);
                            ros::spinOnce();
                            loop_rate.sleep();

                            if(target_angle > bebop_attitude.yaw)
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                bebop_control_message.angular.z = 0.0;
                                bebop_control_from_gps_publisher.publish(bebop_control_message);

                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //bebop_control_message.angular.z = (bebop_attitude.yaw - target_angle)/1.5; // 왼쪽으로 회전
                        bebop_control_message.angular.z = ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                else if(-(1.0/2.0)*M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= -0.0) // yaw state 2
                {
                    //std::cout << "Quadrant 3" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(target_angle - bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < bebop_attitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = -(target_angle - bebop_attitude.yaw)/1.5; // 오른쪽으로 회전
                    bebop_control_message.angular.z = -ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if(-M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= -(1.0/2.0)*M_PI) // yaw state 3
                {
                    //std::cout << "Quadrant 3" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(target_angle - bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < bebop_attitude.yaw) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = -(target_angle - bebop_attitude.yaw)/1.5; // 오른쪽으로 회전
                    bebop_control_message.angular.z = -ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if((1.0/2.0)*M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= M_PI) // yaw state 4
                {
                    //std::cout << "Quadrant 3" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(bebop_attitude.yaw - target_angle < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = DOWN_ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle > bebop_attitude.yaw)
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = (bebop_attitude.yaw - target_angle)/1.5; // 왼쪽으로 회전
                    bebop_control_message.angular.z = ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                continue;
            }//Quadrant 3 end
            else if((go_latitude > current_position.latitude) && (go_longitude < current_position.longitude))//목표위치 기준 4사분면이면
            {
                if(0.0 < bebop_attitude.yaw && bebop_attitude.yaw <= (1.0/2.0)*M_PI) // yaw state 1
                {
                    //std::cout << "Quadrant 4" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(target_angle + bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = DOWN_ANGULAR_SPEED; // 왼쪽 회전
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < fabs(bebop_attitude.yaw)) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = (target_angle + bebop_attitude.yaw)/1.5; // 왼쪽으로 회전
                    bebop_control_message.angular.z = ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();

                }
                else if(-(1.0/2.0)*M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= -0.0) // yaw state 2
                {
                    //std::cout << "Quadrant 4" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(fabs(bebop_attitude.yaw) < target_angle)
                    {
                        if(target_angle + bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                        {
                            bebop_control_message.angular.z = DOWN_ANGULAR_SPEED; // 왼쪽 회전
                            bebop_control_from_gps_publisher.publish(bebop_control_message);
                            ros::spinOnce();
                            loop_rate.sleep();

                            if(target_angle < fabs(bebop_attitude.yaw)) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                bebop_control_message.angular.z = 0.0;
                                bebop_control_from_gps_publisher.publish(bebop_control_message);

                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //bebop_control_message.angular.z = (target_angle + bebop_attitude.yaw)/1.5; // 왼쪽으로 회전
                        bebop_control_message.angular.z = ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    else if(fabs(bebop_attitude.yaw) >= target_angle)
                    {
                        if(fabs(bebop_attitude.yaw) - target_angle < DOWN_ANG_SPEED_GAP)
                        {
                            bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                            bebop_control_from_gps_publisher.publish(bebop_control_message);
                            ros::spinOnce();
                            loop_rate.sleep();

                            if(target_angle > fabs(bebop_attitude.yaw)) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                            {
                                //std::cout << "target angle set end..." << std::endl;
                                bebop_control_message.angular.z = 0.0;
                                bebop_control_from_gps_publisher.publish(bebop_control_message);

                                target_angle_set = false;
                                //std::cout << "target angle set State" << target_angle_set << std::endl;
                            }
                            continue;
                        }
                        //bebop_control_message.angular.z = -(fabs(bebop_attitude.yaw) - target_angle)/1.5; // 오른쪽으로 회전
                        bebop_control_message.angular.z = -ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                else if(-M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= -(1.0/2.0)*M_PI) // yaw state 3
                {
                    //std::cout << "Quadrant 4" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(fabs(bebop_attitude.yaw) - target_angle < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = -DOWN_ANGULAR_SPEED; // 오른쪽 회전
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle > fabs(bebop_attitude.yaw)) // target angle을 지나치면 각속도를 0으로 하고 회전을 멈춘다.
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = -(fabs(bebop_attitude.yaw) - target_angle)/1.5; // 오른쪽으로 회전
                    bebop_control_message.angular.z = -ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                else if((1.0/2.0)*M_PI < bebop_attitude.yaw && bebop_attitude.yaw <= M_PI) // yaw state 4
                {
                    //std::cout << "Quadrant 4" << std::endl;
                    //std::cout << "target angle:" << target_angle << std::endl;
                    //std::cout << "bebop_attitude.yaw:" << bebop_attitude.yaw << std::endl;
                    //std::cout << "target angle setting..." << std::endl;

                    if(target_angle + bebop_attitude.yaw < DOWN_ANG_SPEED_GAP)
                    {
                        bebop_control_message.angular.z = DOWN_ANGULAR_SPEED;
                        bebop_control_from_gps_publisher.publish(bebop_control_message);
                        ros::spinOnce();
                        loop_rate.sleep();

                        if(target_angle < fabs(bebop_attitude.yaw))
                        {
                            //std::cout << "target angle set end..." << std::endl;
                            bebop_control_message.angular.z = 0.0;
                            bebop_control_from_gps_publisher.publish(bebop_control_message);

                            target_angle_set = false;
                            //std::cout << "target angle set State" << target_angle_set << std::endl;
                        }
                        continue;
                    }
                    //bebop_control_message.angular.z = (target_angle + bebop_attitude.yaw)/1.5; // 왼쪽으로 회전
                    bebop_control_message.angular.z = ANGULAR_SPEED;
                    bebop_control_from_gps_publisher.publish(bebop_control_message);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                continue;
            }//Quadrant 4 end
        }//target angle set end


        //회전 후 직진
        if(go_target_gps_set)
        {
            if(calDistance(go_latitude, go_longitude, current_position.latitude, current_position.longitude) > STOP_DISTANCE)
            {
                //bebop_control_message.linear.x = calDistance(go_latitude, go_longitude, current_position.latitude, current_position.longitude)/20.0;
                if(calDistance(go_latitude, go_longitude, current_position.latitude, current_position.longitude) < DOWN_SPEED_DISTANCE)
                {
                  bebop_control_message.linear.x = DOWN_LINEAR_SPEED;
                  bebop_control_from_gps_publisher.publish(bebop_control_message);
                  ros::spinOnce();
                  loop_rate.sleep();
                  continue;
                }
                bebop_control_message.linear.x = LINEAR_SPEED;
                bebop_control_from_gps_publisher.publish(bebop_control_message);

                //std::cout << "bebop speed:" << bebop_control_message.linear.x << std::endl;
                //std::cout << "distance:" << bebop_control_message.linear.x*10.0 << std::endl;

                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
            go_target_gps_set = false;
        }

        bebop_control_message.linear.x = 0.0;
        bebop_control_message.angular.z = 0.0;
        bebop_control_from_gps_publisher.publish(bebop_control_message);

        // if(end_altitude_set)
        // {
        //     if(current_position.altitude > DOWN_ALTITUDE)
        //     {
        //         bebop_control_message.linear.z = -LINEAR_SPEED;
        //         bebop_control_from_gps_publisher.publish(bebop_control_message);
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //         continue;
        //     }
        //     else
        //     {
        //       bebop_control_message.linear.z = 0.0;
        //       bebop_control_from_gps_publisher.publish(bebop_control_message);
        //       end_altitude_set = false;
        //     }
        // }

        //std::cout << "STOP!" << std::endl;
        //node_handle.setParam(go_entered_coordinates_key, end_go_entered_coordinates);
        //std::cout << "END move" << std::endl;
        break;
    }// while end

    bebop_control_message.linear.x = 0.0;
    bebop_control_message.linear.z = 0.0;
    bebop_control_message.angular.z = 0.0;
    bebop_control_from_gps_publisher.publish(bebop_control_message);
    node_handle.setParam(go_entered_coordinates_key, end_go_entered_coordinates);
    //node_handle.getParam(go_entered_coordinates_key, does_go_entered_coordinates);
    //std::cout << "go entered coordinates State:" << does_go_entered_coordinates << std::endl;
    //std::cout << "Not Go Entered Coordinates State!!! DON'T MOVE!!!" << std::endl;
    start_altitude_set = true;
    end_altitude_set = true;
    target_angle_set = true;
    go_target_gps_set = true;


}//_currentGoEnteredCoordinates callback end


void BebopGlobalPositioningSystem::GoSelectedPlace()
{
    int select_place = 0;
    // double ho101_latitude = 48.878858;
    // double ho101_longitude = 2.367835;
    // double ho102_latitude = 48.878836;
    // double ho102_longitude = 2.367694;
    // double ho103_latitude = 48.878919;
    // double ho103_longitude = 2.367665;
    // double ho104_latitude = 48.878943;
    // double ho104_longitude = 2.367783;
    // double ho105_latitude = 48.878973;
    // double ho105_longitude = 2.367867;
    int ho101_id = 9;
    int ho102_id = 10;
    int ho103_id = 11;
    // double ho101_latitude = 36.519493;
    // double ho101_longitude = 127.172462;
    // double ho102_latitude = 36.519515;
    // double ho102_longitude = 127.172792;
    // double ho103_latitude = 36.519743;
    // double ho103_longitude = 127.172996;
    // double ho104_latitude = 36.519826;
    // double ho104_longitude = 127.172736;
    // double ho105_latitude = 36.519743;
    // double ho105_longitude = 127.172354;
    double ho101_latitude = 36.519881;
    double ho101_longitude =  127.173621;
    double ho102_latitude = 36.520071;
    double ho102_longitude = 127.173538;
    double ho103_latitude = 36.520071;
    double ho103_longitude = 127.173538;
    double ho104_latitude = 36.520071;
    double ho104_longitude = 127.173538;
    double ho105_latitude = 36.520071;
    double ho105_longitude = 127.173538;

    node_handle.getParam(select_place_key, select_place);
    if(select_place == 1)
    {
        node_handle.setParam(go_latitude_key, ho101_latitude);
        node_handle.setParam(go_longitude_key, ho101_longitude);
        node_handle.setParam(ar_marker_id_key, ho101_id);
    }
    else if(select_place == 2)
    {
        node_handle.setParam(go_latitude_key, ho102_latitude);
        node_handle.setParam(go_longitude_key, ho102_longitude);
        node_handle.setParam(ar_marker_id_key, ho102_id);
    }
    else if(select_place == 3)
    {
        node_handle.setParam(go_latitude_key, ho103_latitude);
        node_handle.setParam(go_longitude_key, ho103_longitude);
        node_handle.setParam(ar_marker_id_key, ho103_id);
    }
    else if(select_place == 4)
    {
        node_handle.setParam(go_latitude_key, ho104_latitude);
        node_handle.setParam(go_longitude_key, ho104_longitude);
    }
    else if(select_place == 5)
    {
        node_handle.setParam(go_latitude_key, ho105_latitude);
        node_handle.setParam(go_longitude_key, ho105_longitude);
    }

    ControlGps();

}



void BebopGlobalPositioningSystem::GetParam()
{
    node_handle.getParam(take_off_key, does_take_off);//드론이 takeoff 했는지 does_take_off에 상태를 받아옴
    node_handle.getParam(gps_key, is_gps);//드론의 gps를 사용할 준비가 됐는지 isGPS에 상태를 받아옴
    node_handle.getParam(go_home_key, does_go_home); //gps 기능에서 go home이 활성화 됐는지 does_go_home에 상태를 받아옴
    node_handle.getParam(go_entered_coordinates_key, does_go_entered_coordinates);//gps 기능에서 go entered coordinates키가 눌리고 좌표가 입력됬는지 does_go_entered_coordinates에 상태를 받아옴
    node_handle.getParam(go_place_key, does_go_place);//gps 기능에서 return item키가 눌리고 가져올 장소가 선택됬는지 does_go_place에 상태를 받아옴
}

void BebopGlobalPositioningSystem::Action()
{
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        GetParam();

        if(does_go_home)
        {
            ReturnHome();
        }
        else if(does_go_entered_coordinates)
        {
            ControlGps();
        }
        else if(does_go_place)
        {
            GoSelectedPlace();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

//생성자
BebopGlobalPositioningSystem::BebopGlobalPositioningSystem(const ros::NodeHandle& nh)
:node_handle(nh),
bebop_control_from_gps_publisher(node_handle.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10)),
bebop_go_home_publisher(node_handle.advertise<std_msgs::Bool>("/bebop/autoflight/navigate_home", 10)),
land_publisher(node_handle.advertise<std_msgs::Empty>("bebop/land", 1)),
current_gps_subscriber(node_handle.subscribe(bebop_position_changed, 1, &BebopGlobalPositioningSystem::CurrentGpsCallback, this)),
current_attitude_subscriber(node_handle.subscribe(bebop_attitude_changed, 1, &BebopGlobalPositioningSystem::CurrentAttitudeCallback, this)),
does_take_off(false),
is_gps(false),
does_go_home(false),
does_go_entered_coordinates(false),
does_go_place(false)
{

}
