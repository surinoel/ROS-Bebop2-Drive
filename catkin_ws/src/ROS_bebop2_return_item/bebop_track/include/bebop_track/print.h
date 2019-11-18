#ifndef PRINT_H
#define PRINT_H

void printSpeed(double speedX, double speedY, double speedZ, double speedYaw){
  ROS_INFO("[SPEED] X : %lf", speedX);
  ROS_INFO("[SPEED] Y : %lf", speedY);
  ROS_INFO("[SPEED] Z : %lf", speedZ);
  ROS_INFO("[SPEED] YAW : %lf", speedYaw);
}

void printHomeGPS(double gpsX, double gpsY, double gpsZ){
  ROS_INFO("[HOME] LONGITUDE : %lf", gpsX);  // x
  ROS_INFO("[HOME] LATITUDE : %lf", gpsY);  // y
  ROS_INFO("[HOME] ALTITUDE : %lf", gpsZ);  // z
}

void printDroneGPS(double gpsX, double gpsY, double gpsZ){
  ROS_INFO("[DRONE] LONGITUDE : %lf", gpsX);  // x
  ROS_INFO("[DRONE] LATITUDE : %lf", gpsY);  // y
  ROS_INFO("[DRONE] ALTITUDE : %lf", gpsZ);  // z
}

void printRotation(double roll, double pitch, double yaw){
  ROS_INFO("[DRONE] ROLL : %lf", roll);
	ROS_INFO("[DRONE] PITCH : %lf", pitch);
	ROS_INFO("[DRONE] YAW : %lf", yaw);
}

void printGap(double distance, double longitude_gap, double latitude_gap, double altitude_gap, double theta){
  ROS_INFO("[DISTANCE] %lf", distance);
  ROS_INFO("[GAP] X : %lf", longitude_gap);
  ROS_INFO("[GAP] Y : %lf", latitude_gap);
  ROS_INFO("[GAP] Z : %lf", altitude_gap);
  ROS_INFO("[GAP] THETA : %lf", theta);
}

#endif
