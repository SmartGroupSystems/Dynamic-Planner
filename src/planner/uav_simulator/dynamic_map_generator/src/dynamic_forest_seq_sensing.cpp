/**
 * @file dynamic_forest_seq_sensing.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief generate a sequence of dynamic forest maps for fake sensing
 * To generate the current global map and local map in next time steps,
 * we need to publish current global point cloud and velocity
 *
 * The cylinder velocity is published as the difference between current position
 * and next position, which is (marker.points[1] - marker.points[0])
 *
 * @version 1.0
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

// for cylinders
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// for dynamic obstacles
#include <Eigen/Eigen>
#include <random>

#include "map_generator/moving_circle.h"
#include "map_generator/moving_cylinder.h"

using namespace std;

vector<int>   pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device                     rd;
default_random_engine             eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;

// ros::Publisher _local_map_pub;
ros::Publisher _all_map_cloud_pub, _all_map_cylinder_pub, _all_map_cylinder_pub_vis;
ros::Publisher click_map_pub_, _cylinder_state_pub;

vector<double> _state;

int         _obs_num, _circle_num;
double      _x_size, _y_size, _z_size;
double      _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _v_h, _dr;
double      _radius_h, _radius_l, _z_l, _z_h, _theta, _omega_h;
double      _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
std::string _frame_id;

bool _map_ok       = false;
bool _has_odom     = false;
bool _set_cylinder = false;
bool _test_mode    = false;

/**@brief map mode
 * 0: randomize both vx and vy
 * 1: randomize vx, vy = 0
 * 2: randomize vy, vx = 0
 */
int _mode = 0;

/* map sequence settings */
bool   _future_map = false;
int    _num_future_map;
double _future_step_size;

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 globalCylinders_pcd;

pcl::PointCloud<pcl::PointXYZ> clouds;

visualization_msgs::MarkerArray cylinders_vis;
visualization_msgs::Marker      cylinder_mk;
visualization_msgs::MarkerArray circle_vis;
visualization_msgs::Marker      circle_mk;
visualization_msgs::MarkerArray obstacle_state_list;
visualization_msgs::Marker      obstacle_state;

std::vector<dynamic_map_objects::MovingCylinder> _dyn_cylinders;
std::vector<dynamic_map_objects::MovingCircle>   _dyn_circles;

/**
 * @brief generate random map
 *
 */
void RandomMapGenerate() {
  pcl::PointXYZ       pt_random;
  geometry_msgs::Pose pt;
  pt.orientation.w = 1.0;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  // generate pillar obstacles
  _dyn_cylinders.clear();
  _dyn_cylinders.reserve(_obs_num);
  for (int i = 0; i < _obs_num; i++) {
    dynamic_map_objects::MovingCylinder cylinder(_x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h,
                                                 _v_h, eng, _resolution);
    cylinder.setMode(_mode);
    _dyn_cylinders.push_back(cylinder);
  }

  _dyn_circles.clear();
  _dyn_circles.reserve(_circle_num);
  for (int i = 0; i < _circle_num; i++) {
    dynamic_map_objects::MovingCircle circle(_x_l, _x_h, _y_l, _y_h, _z_l, _z_h, _radius_l,
                                             _radius_h, _dr, _v_h, eng, _resolution);
    _dyn_circles.push_back(circle);
  }

  ROS_WARN("Finished generate random map ");

  _map_ok = true;
}

/**
 * @brief
 *
 */
void pubSensedPoints() {
  // concatenate all points
  clouds.points.clear();
  clouds.points.reserve(_obs_num);
  cylinders_vis.markers.clear();
  cylinders_vis.markers.reserve(_obs_num);
  obstacle_state_list.markers.clear();
  obstacle_state_list.markers.reserve(_obs_num);
  cylinder_mk.header.stamp = ros::Time::now();
  cylinder_mk.id           = 0;

  obstacle_state.header.stamp = ros::Time::now();
  obstacle_state.points.clear();
  obstacle_state.id = 0;

  pcl::PointCloud<pcl::PointXYZ> cloud_all;

  for (auto& dyn_cld : _dyn_cylinders) {
    if (!_test_mode) {
      dyn_cld.update();
    }
    cloud_all += dyn_cld._cloud;

    // publish cylinder markers
    pcl::PointXYZ pt_center;
    pt_center.x = dyn_cld.x;
    pt_center.y = dyn_cld.y;
    pt_center.z = dyn_cld.w;
    clouds.points.push_back(pt_center);

    geometry_msgs::Pose pose;
    pose.position.x    = dyn_cld.x;
    pose.position.y    = dyn_cld.y;
    pose.position.z    = 0.5 * dyn_cld.h;
    pose.orientation.w = 1.0;

    cylinder_mk.pose    = pose;
    cylinder_mk.scale.x = cylinder_mk.scale.y = dyn_cld.w;  // less then 1
    cylinder_mk.scale.z                       = dyn_cld.h;
    cylinders_vis.markers.push_back(cylinder_mk);
    cylinder_mk.id += 1;

    obstacle_state.pose               = pose;
    obstacle_state.pose.position.x    = dyn_cld.x;
    obstacle_state.pose.position.y    = dyn_cld.y;
    obstacle_state.pose.position.z    = 0.5 * dyn_cld.h;
    obstacle_state.pose.orientation.w = 1.0;
    obstacle_state.points.clear();
    geometry_msgs::Point pts;
    pts.x = pose.position.x;
    pts.y = pose.position.y;
    pts.z = pose.position.z;
    obstacle_state.points.push_back(pts);
    pts.x += dyn_cld.vx * _sense_rate;
    pts.y += dyn_cld.vy * _sense_rate;
    obstacle_state.points.push_back(pts);
    obstacle_state.scale.x = dyn_cld.w;
    obstacle_state.scale.y = dyn_cld.w;
    obstacle_state.type    = visualization_msgs::Marker::CYLINDER;
    obstacle_state_list.markers.push_back(obstacle_state);
    obstacle_state.id += 1;
  }

  for (auto& dyn_crl : _dyn_circles) {
    if (!_test_mode) {
      dyn_crl.update();
    }
    cloud_all += dyn_crl._cloud;
    pcl::PointXYZ pt_center;
    pt_center.x = dyn_crl.x;
    pt_center.y = dyn_crl.y;
    pt_center.z = dyn_crl.z;
    clouds.points.push_back(pt_center);

    geometry_msgs::Pose pose;
    pose.position.x    = dyn_crl.x;
    pose.position.y    = dyn_crl.y;
    pose.position.z    = dyn_crl.z;
    pose.orientation.w = dyn_crl.q.w();
    pose.orientation.x = dyn_crl.q.x();
    pose.orientation.y = dyn_crl.q.y();
    pose.orientation.z = dyn_crl.q.z();

    cylinder_mk.pose    = pose;
    cylinder_mk.scale.x = 2 * dyn_crl.r2;
    cylinder_mk.scale.y = 2 * dyn_crl.r1;
    cylinder_mk.scale.z = 0.1;
    cylinders_vis.markers.push_back(cylinder_mk);
    cylinder_mk.id += 1;

    obstacle_state.pose.position.x    = dyn_crl.x;
    obstacle_state.pose.position.y    = dyn_crl.y;
    obstacle_state.pose.position.z    = dyn_crl.z;
    obstacle_state.pose.orientation.w = dyn_crl.q.w();
    obstacle_state.pose.orientation.x = dyn_crl.q.x();
    obstacle_state.pose.orientation.y = dyn_crl.q.y();
    obstacle_state.pose.orientation.z = dyn_crl.q.z();

    obstacle_state.points.clear();
    geometry_msgs::Point pts;
    pts.x = pose.position.x;
    pts.y = pose.position.y;
    pts.z = pose.position.z;
    obstacle_state.points.push_back(pts);
    pts.x += dyn_crl.vx * _sense_rate;
    pts.y += dyn_crl.vy * _sense_rate;
    obstacle_state.points.push_back(pts);
    obstacle_state.scale.x = 2 * dyn_crl.r2;
    obstacle_state.scale.y = 2 * dyn_crl.r1;
    obstacle_state.scale.z = 0.1;
    obstacle_state.type    = visualization_msgs::Marker::SPHERE;
    obstacle_state_list.markers.push_back(obstacle_state);
    obstacle_state.id += 1;
  }

  cloud_all.width    = cloud_all.points.size();
  cloud_all.height   = 1;
  cloud_all.is_dense = true;

  // ROS_WARN_STREAM("Publishing " << cloud_all.points.size() << " points");
  // ROS_WARN_STREAM("Publishing " << cylinders.points.size() << " cylinders");
  // ROS_WARN_STREAM("Publishing " << cylinders_vis.markers.size() << " cylinders markers");

  // publish cloud
  pcl::toROSMsg(cloud_all, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(globalMap_pcd);

  // publish cylinder markers
  pcl::toROSMsg(clouds, globalCylinders_pcd);
  globalCylinders_pcd.header.frame_id = _frame_id;
  _all_map_cylinder_pub.publish(globalCylinders_pcd);

  _all_map_cylinder_pub_vis.publish(cylinders_vis);
  _cylinder_state_pub.publish(obstacle_state_list);
  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_map_sequence_sensing");
  ros::NodeHandle n("~");

  //_local_map_pub = n.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
  _all_map_cloud_pub    = n.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  _all_map_cylinder_pub = n.advertise<sensor_msgs::PointCloud2>("global_cylinders", 1);
  _all_map_cylinder_pub_vis =
      n.advertise<visualization_msgs::MarkerArray>("global_cylinders_vis", 1);
  _cylinder_state_pub = n.advertise<visualization_msgs::MarkerArray>("global_cylinder_state", 1);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);
  n.param("map/future", _future_map, true);
  n.param("map/future_num", _num_future_map, 6);
  n.param("map/time_step", _future_step_size, 0.2);
  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/test", _test_mode, false);

  // clearance for multi robots.
  _x_size -= 2.0;
  _y_size -= 2.0;

  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/frame_id", _frame_id, string("map"));

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);
  n.param("ObstacleShape/upper_vel", _v_h, 0.1);
  n.param("ObstacleShape/set_cylinder", _set_cylinder, false);

  n.param("map/circle_num", _circle_num, 0);
  n.param("ObstacleShape/radius_l", _radius_l, 7.0);
  n.param("ObstacleShape/radius_h", _radius_h, 7.0);
  n.param("ObstacleShape/z_l", _z_l, 7.0);
  n.param("ObstacleShape/z_h", _z_h, 7.0);
  n.param("ObstacleShape/dr", _dr, 0.2);
  n.param("ObstacleShape/theta", _theta, 7.0);
  n.param("ObstacleShape/omega", _omega_h, 2.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);
  n.param("mode", _mode, 0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  cylinder_mk.header.frame_id = _frame_id;
  cylinder_mk.type            = visualization_msgs::Marker::CYLINDER;
  cylinder_mk.action          = visualization_msgs::Marker::ADD;
  cylinder_mk.id              = 0;
  cylinder_mk.color.r         = 0.5;
  cylinder_mk.color.g         = 0.5;
  cylinder_mk.color.b         = 0.5;
  cylinder_mk.color.a         = 0.6;

  obstacle_state.header = cylinder_mk.header;
  obstacle_state.type   = visualization_msgs::Marker::ARROW;

  ros::Duration(0.5).sleep();

  RandomMapGenerate();

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    // delete old cylinders in rviz
    // visualization_msgs::Marker delete_cylinders;
    // delete_cylinders.header.frame_id = _frame_id;
    // delete_cylinders.action          = visualization_msgs::Marker::DELETEALL;
    // cylinders_vis.markers.clear();
    // cylinders_vis.markers.push_back(delete_cylinders);
    // _all_map_cylinder_pub_vis.publish(cylinders_vis);

    // update map
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
