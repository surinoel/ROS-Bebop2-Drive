#include <bebop_track/bebop_ar_track.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_ar_track_node");
    ros::NodeHandle nh;

    BebopArMarkerTrack bebop_and_ar_marker(nh);

    bebop_and_ar_marker.Action();

    return 0;
}
