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
#endif
