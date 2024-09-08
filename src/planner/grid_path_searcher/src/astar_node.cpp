/*
输入：
输出：
*/
#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include "Astar_searcher_2d.h"
// #include <grid_path_searcher/PathInitState.h>
// #include "JPS_searcher.h"
//#include "backward.hpp"
#include <sensor_msgs/point_cloud_conversion.h>


using namespace std;
using namespace Eigen;

// namespace backward {
// backward::SignalHandling sh;
// }
/*----------------------global variables----------------------*/
// Simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size;    

bool _has_map   = false;
bool _has_target   = false;
bool enable_flag = true;
double current_seq = 0, sight_radius, interval;
Vector2d _start_pt,current_vel,current_acc;
Vector2d _target_pt;
double actual_height = 0.8;
// Map size
Vector2d _map_lower, _map_upper;
int _max_x_id, _max_y_id;
/*----------------------ROS----------------------*/
// SubPub
ros::Subscriber _map_sub,
                                 _pts_sub,
                                 _nav_sub,
                                 _plan_sub,//读地图与目标点
                                 _fsm_sub;
ros::Publisher  _grid_path_vis_pub, 
                               _visited_nodes_vis_pub,
                               _grid_map_vis_pub,
                               _grid_path_pub,
                               _grid_twist_pub,
                               _grid_pathstate_pub,
                               _arrived_pub;

// Callback
void rcvWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr & pointcloud_map_raw);
void simPoseCallback(const mavros_msgs::PositionTarget & aim_msg);
/*----------------------A*----------------------*/
AstarPathFinder2d * _astar_path_finder     = new AstarPathFinder2d();
bool pathFinding(const Vector2d start_pt, const Vector2d target_pt, 
                 const Vector2d vel_,     const Vector2d acc_);
/*----------------------visual----------------------*/
void visGridPath( vector<Vector2d> nodes, bool is_use_jps );                                            //可视化函数
void visVisitedNode( vector<Vector2d> nodes );           
/*----------------------message pub----------------------*/
void pubGridPath(vector<Vector2d> nodes);
void pubGridPath2(vector<Vector2d> nodes);
void pubGridTwist(vector<Vector2d> nodes);
void pubGridTwist2(vector<Vector2d> nodes);
void pubGridTwist3(vector<Vector2d> nodes, Vector2d vel_, Vector2d acc_);
/*----------------------function----------------------*/
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr & pointcloud_map_raw)
{   
    // cout<<"enable_flag"<<enable_flag<<endl;
    if(!enable_flag) return;

    sensor_msgs::PointCloud2 pointcloud_map = *pointcloud_map_raw;
    // convertPointCloudToPointCloud2(*pointcloud_map_raw, pointcloud_map);
    if(_has_map){
        _astar_path_finder->cleanObs();
        //  cout<<"Has map, clean all."<<endl;
    }
     
    pcl::PointCloud<pcl::PointXYZ> cloud;           //容器填充为三维点
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;
     
    pcl::fromROSMsg(pointcloud_map, cloud);//类型变换
     
    if( (int)cloud.points.size() == 0 ) return;//空地图则返回
     
    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)//遍历点云中的点
    {
        //FIXME 输入点云是三维的，这里需要确定z的范围
        //QUES 是不是0上下的一个范围？
        // if(高度不在范围内) continue;
        pt = cloud.points[idx];        //取点到pt
        //FIXME 修改0.5为目标高度范围
        if(std::abs(pt.z-actual_height)>_resolution) continue;
         _astar_path_finder->setObs(pt.x, pt.y);
        // for visualize only
        // 可视化
        Vector2d cor_round = _astar_path_finder->coordRounding(Vector2d(pt.x, pt.y));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = 0;
        cloud_vis.points.push_back(pt);
        Vector2i center_index = _astar_path_finder -> coord2gridIndex(Vector2d(pt.x, pt.y));
        Vector2d pt_around_d;
        Vector2i  pt_around_i;
        for(int row_index = -1;row_index<=1;row_index++){
            for(int col_index = -1;col_index<=1;col_index++){
                if(row_index == 0 && col_index == 0) continue;
                pt_around_i = center_index + Vector2i(row_index,col_index);
                pt_around_d = _astar_path_finder -> gridIndex2coord(pt_around_i);
                if(_astar_path_finder -> getData(pt_around_i)) {
                    pt.x = pt_around_d(0);
                    pt.y = pt_around_d(1);
                    cloud_vis.points.push_back(pt);
                }
            }
        }
        
        
        // 可视化end
    }
     
    // 可视化
    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);  //将处理好的可视化数据发布到ROS话题中
    map_vis.header.frame_id = "world";
    _grid_map_vis_pub.publish(map_vis);//demo_node/grid_map_vis
    // 可视化end

    _has_map = true;
    // ROS_INFO_DELAYED_THROTTLE(1,"[Astar] Pointcloud received.");
    // ROS_INFO("[Astar] Pointcloud received.");
    std_msgs::Int64 arr_msg;

    if(!_has_target)
    {
        // cout << "\033[33m[Astar] get!\033[0m" << endl;
    }else  
    {
        bool start_warn_flag = true,end_warn_flag = true;
        if(_astar_path_finder->arrived(_start_pt,_target_pt)) // 到达目标点
        {

            cout << "\033[32m[Astar] Arrived!\n\033[31m[Astar] PathFinding will not run! \033[0m" << endl;
            arr_msg.data = 1;
            _arrived_pub.publish(arr_msg);
        }
        else
        {
            if(_astar_path_finder->getData(_start_pt)) 
            {
                // TODO 修改起点推出条件
                cout << "\033[33m[Astar] _start_pt is Occupied! Reset Obs. BE CAREFUL!\033[0m" << endl;
                _astar_path_finder -> cleanStartObs(_start_pt);
                // if(1)
                // {
                start_warn_flag = false;
                // }
            }
            else
            {
                start_warn_flag = false;
            }
            if(_astar_path_finder -> getData(_target_pt)) // 目标点被占据
            {
                std::pair<bool,Eigen::Vector2i> camecame_need;
                camecame_need = _astar_path_finder -> check_collision(_target_pt, _start_pt);
                if(camecame_need.first) // 不存在可达点
                {
                    cout << "\033[31m[Astar] _target_pt is Occupied! PathFinding will not run!\033[0m" << endl;
                }
                else
                {
                    cout << "\033[33m[Astar] Get the nearest non-occupied _target_pt.\033[0m" << endl;
                    end_warn_flag = false;
                    _target_pt = _astar_path_finder -> gridIndex2coord(camecame_need.second);
                }
            } 
            else
            {
                end_warn_flag = false;
            }
            if(!start_warn_flag && !end_warn_flag)
            {
                if(!pathFinding(_start_pt, _target_pt,current_vel,current_acc)) 
                cout << "\033[31m[Astar] No path provide! \033[0m" << endl;
                else{
                cout << "\033[32m[Astar] Complete the search! \033[0m" << endl;
                    arr_msg.data = -1;
                    _arrived_pub.publish(arr_msg);
                }
            }
            else
            {
                cout << "\033[31m[Astar] PathFinding will not run! \033[0m" << endl;
            }
        }

        
    }


}

void rcvWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr & wp)
{     
    //USAGE 拿到waypoint消息后，检查并进行路径寻找
    //起点为全局变量_start_pt，终点为消息中的waypoint
    if( _has_map == false ) //无地图，退出函数
    {
        ROS_WARN("[Astar] No map! ");
        // return;
    }
    cout<<""<<endl;
    // Vector2d _target_pt;
    _target_pt << wp->pose.position.x,
                  wp->pose.position.y;
    _has_target = true;
    
}

void simPoseCallback(const mavros_msgs::PositionTarget & aim_msg)
{
    _start_pt[0]  = aim_msg.position.x;
    _start_pt[1]  = aim_msg.position.y;
    current_vel[0] = aim_msg.velocity.x;
    current_vel[1] = aim_msg.velocity.y;
    current_acc[0] = aim_msg.acceleration_or_force.x;
    current_acc[1] = aim_msg.acceleration_or_force.y;
    current_seq = (int)aim_msg.yaw;
    actual_height = aim_msg.position.z; // BUG
    _astar_path_finder->start_pt_real = _start_pt;

}

void fsmCallback(const std_msgs::Int64::ConstPtr & msg)
{
    if(msg->data == 3) 
        enable_flag = true;
    else
        enable_flag = false;
    // cout <<"flag_"<<msg->data<<"received!"<<endl;
}

bool pathFinding(const Vector2d start_pt, const Vector2d target_pt, 
                 const Vector2d vel_,     const Vector2d acc_)
{
    //Call A* to search for a path
    cout<<"[Astar] PathFinding start."<<endl;
    if(!_astar_path_finder->AstarGraphSearch(start_pt, target_pt)) return 0;

    //Retrieve the path
    auto grid_twist    = _astar_path_finder->getTwist_checkcolision3(start_pt);
    // auto visited_nodes = _astar_path_finder->getVisitedNodes();

    // cout <<"\033[46m--------------------grid_path - grid_twist--------------------"<<endl;
    // auto ptr_old = *(grid_twist.begin());
    // for(auto ptr : grid_twist)
    // {
    // cout << Eigen::Vector2d(ptr - ptr_old).norm() <<endl;
    // ptr_old = ptr;
    // }
    // cout <<"----------------------------------------\033[0m"<<endl;
    //Visualize the result
    cout <<"(*(grid_twist.begin()))"<< (*(grid_twist.begin())) << endl;
    visGridPath(grid_twist, false);
    // visVisitedNode(visited_nodes);
    pubGridPath(grid_twist);
    // pubGridTwist(grid_twist);
    // pubGridTwist2(grid_twist);
    pubGridTwist3(grid_twist,vel_,acc_);

    //Reset map for next call
    _astar_path_finder->resetUsedGrids();
    return 1;
}
/*----------------------main----------------------*/
int main(int argc, char** argv)
{
//init
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle nh("~");

//sub
    _map_sub = nh.subscribe( "map",  10, rcvPointCloudCallBack );
    _pts_sub = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _nav_sub = nh.subscribe( "pose", 1, simPoseCallback );
    _fsm_sub = nh.subscribe( "/flag_detect",1,fsmCallback);
//pub
    _grid_map_vis_pub      = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub     = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
    _grid_path_pub         = nh.advertise<nav_msgs::Path>("grid_path",1);
    _grid_twist_pub        = nh.advertise<nav_msgs::Path>("grid_twist",1);
    _arrived_pub           = nh.advertise<std_msgs::Int64>("target_arrived",1);
    

    nh.param("map/resolution",    _resolution,   0.2);
    nh.param("map/x_size"         ,        _x_size, 60.0);
    nh.param("map/y_size"         ,        _y_size, 60.0);    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);//QUES 是否每次从0开始运行？
    nh.param("planning/sight_radius",  sight_radius,  5.0);
    nh.param("planning/interval",  interval,  0.5);

    _map_lower  << - _x_size/2.0,  -  _y_size/2.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0;
    _inv_resolution = 1.0 / _resolution;
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);

    _astar_path_finder  = new AstarPathFinder2d();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, interval);
    
//ROS
    ros::Rate rate(50.0);
    while(ros::ok()) 
    {
        // ROS_INFO("test");
        ros::spinOnce();
        // rate.sleep();
    }
    delete _astar_path_finder;
    return 0;
}

void pubGridPath(vector<Vector2d> nodes){
    nav_msgs::Path node_nav;
    node_nav.header.frame_id = "world";
    geometry_msgs::PoseStamped pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.header.seq = i+1;
        pt.pose.position.x = coord(0);
        pt.pose.position.y = coord(1);
        pt.pose.position.z =  0;

        node_nav.poses.push_back(pt);
    }
    
    _grid_path_pub.publish(node_nav);
    
}
void pubGridPath2(vector<Vector2d> nodes){
    nav_msgs::Path node_nav;
    node_nav.header.frame_id = "world";
    geometry_msgs::PoseStamped pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.header.seq = i+1;
        pt.pose.position.x = coord(0);
        pt.pose.position.y = coord(1);
        pt.pose.position.z =  0;

        node_nav.poses.push_back(pt);
    }
    
    _grid_path_pub.publish(node_nav);
    
}
void pubGridTwist(vector<Vector2d> nodes){
    nav_msgs::Path node_nav;
    node_nav.header.frame_id = "world";
    geometry_msgs::PoseStamped pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.header.seq = i+1;
        pt.pose.position.x = coord(0);
        pt.pose.position.y = coord(1);
        pt.pose.position.z =  0;

        node_nav.poses.push_back(pt);
    }
    _grid_twist_pub.publish(node_nav);

    
}

void pubGridTwist2(vector<Vector2d> nodes){
    nav_msgs::Path node_nav;
    node_nav.header.frame_id = "world";
    geometry_msgs::PoseStamped pt;
    Vector2d start_coord_ = nodes[0];
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord     = nodes[i];
        Vector2d d_coord = coord - start_coord_;
        if(d_coord.norm()>=7.0) break; // BUG
        pt.header.seq      = i+1;
        pt.pose.position.x = coord(0);
        pt.pose.position.y = coord(1);
        pt.pose.position.z =  0;

        node_nav.poses.push_back(pt);
    }
    _grid_twist_pub.publish(node_nav);
}

void pubGridTwist3(vector<Vector2d> nodes ,Vector2d vel_, Vector2d acc_){
    nav_msgs::Path node_nav;
    node_nav.header.frame_id = "world";

    // V and A
    geometry_msgs::PoseStamped h_pt;
    h_pt.pose.position.x = vel_[0];
    h_pt.pose.position.y = vel_[1];
    h_pt.pose.position.z = current_seq;
    node_nav.poses.push_back(h_pt);
    h_pt.pose.position.x = acc_[0];
    h_pt.pose.position.y = acc_[1];
    h_pt.pose.position.z = 0;
    node_nav.poses.push_back(h_pt);

    geometry_msgs::PoseStamped pt;
    Vector2d start_coord_ = nodes[0];
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord     = nodes[i];
        Vector2d d_coord = coord - start_coord_;
        if(d_coord.norm()>=sight_radius) break;
        pt.header.seq      = i+1;
        pt.pose.position.x = coord(0);
        pt.pose.position.y = coord(1);
        pt.pose.position.z =  0;

        node_nav.poses.push_back(pt);
    }
    _grid_twist_pub.publish(node_nav);
}

void visGridPath( vector<Vector2d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    
    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < 1; i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = 0;

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode( vector<Vector2d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = 0;

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}