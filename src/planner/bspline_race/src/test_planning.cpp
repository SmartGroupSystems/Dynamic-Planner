#include <ros/ros.h>
#include <bspline_race/bspline_race.h>

using namespace FLAG_Race;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Test_planning");
    ros::NodeHandle nh("~");
    plan_manager manager(nh);
    // geometry_msgs::PoseStamped s,e;
    // s.pose.position.x=0.0; s.pose.position.y=0.0;
    // e.pose.position.x=10.0; e.pose.position.y=12.0;
    // cout<<manager.getSmoothTraj(s,e)<<endl;
    ros::Rate rate(10.0);
    ros::spin();
    return 0;
}

