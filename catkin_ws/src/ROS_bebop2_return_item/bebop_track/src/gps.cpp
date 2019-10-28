#include <bebop_track/bebop_gps.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_gps_node");
    ros::NodeHandle nodeHandle;
    //ros::MultiThreadedSpinner spinner(4);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    
    BebopGlobalPositioningSystem bebopGPS(nodeHandle);

    //ros::spin();
    //spinner.spin();    
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
