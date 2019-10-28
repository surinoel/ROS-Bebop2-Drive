// http://egloos.zum.com/metashower/v/313035

#include <math.h>
#include "control.h"

double calDistance(double lat1, double lon1, double lat2, double lon2){

    double theta, dist;
    theta = lon1 - lon2;
    dist = sin(degreeToRadian(lat1)) * sin(degreeToRadian(lat2)) + cos(degreeToRadian(lat1))
          * cos(degreeToRadian(lat2)) * cos(degreeToRadian(theta));
    dist = acos(dist);
    dist = radianToDegree(dist);

    dist = dist * 60 * 1.1515;
    dist = dist * 1.609344;    // 단위 mile 에서 km 변환.
    dist = dist * 1000.0;      // 단위  km 에서 m 로 변환

    return dist;
}
