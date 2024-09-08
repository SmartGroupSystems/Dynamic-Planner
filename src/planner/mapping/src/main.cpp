#include "grid_map.cpp"
#include "raycast.cpp"

int main(int argc, char  **argv)
{
    /* 初始化ROS节点 */
    ros::init(argc, argv, "Main_N");
    ros::NodeHandle main_n("~");

    GridMap::Ptr gridmap;
	gridmap.reset(new GridMap);
    gridmap->initMap(main_n);

    ros::spin();

    return 0;
}



