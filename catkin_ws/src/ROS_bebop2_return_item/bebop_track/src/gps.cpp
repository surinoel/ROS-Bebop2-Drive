#include <bebop_track/bebop_gps.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_gps_node");
    ros::NodeHandle nh;

    BebopGlobalPositioningSystem bebop_gps(nh);

    bebop_gps.Action();

    return 0;
}
