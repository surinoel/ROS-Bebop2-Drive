#include "test.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "bebop_test");
	ros::NodeHandle nh;

  //CONTROL bebop_control;
	bebop_control = new CONTROL;

	bebop_control->readyToPublish(nh);
  bebop_control->subscribers(nh);

	ros::Rate loopRate(50);

	ros::Timer inputKey = nh.createTimer(ros::Duration(0.01), controlKeyboard);

  while(ros::ok()){
			ros::spinOnce();
			loopRate.sleep();
  }

	return 0;
}
