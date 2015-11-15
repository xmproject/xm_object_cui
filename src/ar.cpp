#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ar");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/ar_pose_marker/markers[0]", 1000, Callback);
    ros::spin();

  return 0;
}
}
