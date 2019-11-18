#ifndef __GPS_TO_XY_H__
#define __GPS_TO_XY_H__

#include <math.h>

/////////////////////////////////////////////
double degree_to_radian(double degree){
	return degree * (M_PI / 180);
}

double radian_to_degree(double radian){
	return radian * (180 / M_PI);
}
/////////////////////////////////////////////
//현재 드론이 위치한 위도,경도와 드론이 가야하는 위도,경도를 가지고 거리를 계산하는 함수 
double calDistance(double lat1, double lon1, double lat2, double lon2){

    double theta, dist;
    theta = lon1 - lon2;
    dist = sin(degree_to_radian(lat1)) * sin(degree_to_radian(lat2)) + cos(degree_to_radian(lat1))
          * cos(degree_to_radian(lat2)) * cos(degree_to_radian(theta));
    dist = acos(dist);
    dist = radian_to_degree(dist);

    dist = dist * 60 * 1.1515;
    dist = dist * 1.609344;    // 단위 mile 에서 km 변환.
    dist = dist * 1000.0;      // 단위  km 에서 m 로 변환

    return dist;
}
////////////////////////////////////////////
//현재 드론이 위치한 위도,경도가 드론이 가야하는 위도 경도를 기준으로 몇도 꺾여있는지 구하는 함수.
//왼쪽으로 0.0 ~ 3.141592 rad 값이 나오고 오른쪽으로 0.0 ~ 3.141592 rad의 값이 나온다. bebop의 yaw값과 다르게 왼쪽 오른쪽 둘 다 양수가 나온다.
double bearingP1toP2(double P1_latitude, double P1_longitude, double P2_latitude, double P2_longitude)
{
    // 현재 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 라디안 각도로 변환한다.
    double Cur_Lat_radian = P1_latitude * (3.141592 / 180);
    double Cur_Lon_radian = P1_longitude * (3.141592 / 180);


    // 목표 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 라디안 각도로 변환한다.
    double Dest_Lat_radian = P2_latitude * (3.141592 / 180);
    double Dest_Lon_radian = P2_longitude * (3.141592 / 180);

    // radian distance
    double radian_distance = 0;
    radian_distance = acos(sin(Cur_Lat_radian) * sin(Dest_Lat_radian) + cos(Cur_Lat_radian) * cos(Dest_Lat_radian) * cos(Cur_Lon_radian - Dest_Lon_radian));

    // 목적지 이동 방향을 구한다.(현재 좌표에서 다음 좌표로 이동하기 위해서는 방향을 설정해야 한다. 라디안값이다.
    double radian_bearing = acos((sin(Dest_Lat_radian) - sin(Cur_Lat_radian) * cos(radian_distance)) / (cos(Cur_Lat_radian) * sin(radian_distance)));        // acos의 인수로 주어지는 x는 360분법의 각도가 아닌 radian(호도)값이다.        
    
    /*double true_bearing = 0;
    if (sin(Dest_Lon_radian - Cur_Lon_radian) < 0)
    {
        true_bearing = radian_bearing * (180 / 3.141592);
        true_bearing = 360 - true_bearing;
    }
    else
    {
        true_bearing = radian_bearing * (180 / 3.141592);
    }*/

    //return true_bearing; //unit: degree
    return radian_bearing; //unit: radian
}

#endif
