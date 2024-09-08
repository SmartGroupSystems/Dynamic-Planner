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
ros::Publisher click_map_pub_;

vector<double> _state;

int         _obs_num;
double      _x_size, _y_size, _z_size;
double      _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _v_h;
double      _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
std::string _frame_id;

bool _map_ok       = false;
bool _has_odom     = false;
bool _set_cylinder = false;

int                               circle_num_;
double                            radius_l_, radius_h_, z_l_, z_h_;
double                            theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 globalCylinders_pcd;

pcl::PointCloud<pcl::PointXYZ> cloudMap;
pcl::PointCloud<pcl::PointXYZ> cylinders;

visualization_msgs::MarkerArray cylinders_vis;
visualization_msgs::Marker      cylinder_mk;

std::vector<dynamic_map_objects::MovingCylinder> _dyn_cylinders;

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

  rand_radius_  = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_   = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_       = uniform_real_distribution<double>(z_l_, z_h_);

  // generate pillar obstacles
  _dyn_cylinders.clear();
  _dyn_cylinders.reserve(_obs_num);
  for (int i = 0; i < _obs_num; i++) {
    dynamic_map_objects::MovingCylinder cylinder(_x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _v_h,
                                                 eng, _resolution);
    _dyn_cylinders.push_back(cylinder);
  }

  // generate circle obstacles
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    // random point in grid scale
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    // Homogeneous transformation
    Eigen::Vector3d translate(x, y, z);
    double          theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution, ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width    = cloudMap.points.size();
  cloudMap.height   = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  _map_ok = true;
}

/**
 * @brief
 *
 */
void pubSensedPoints() {
  // concatenate all points
  pcl::PointCloud<pcl::PointXYZ> cloud_all;
  cylinders.clear();
  cylinders.points.reserve(_obs_num);
  cylinders_vis.markers.clear();
  cylinders_vis.markers.reserve(_obs_num);
  cylinder_mk.header.stamp = ros::Time::now();
  
  cloud_all += cloudMap;
  for (auto& dyn_cld : _dyn_cylinders) {
    dyn_cld.update();
    cloud_all += dyn_cld._cloud;

    // publish cylinder markers
    pcl::PointXYZ pt_center;
    pt_center.x = dyn_cld.x;
    pt_center.y = dyn_cld.y;
    pt_center.z = dyn_cld.w;
    cylinders.points.push_back(pt_center);

    geometry_msgs::Pose pose;
    pose.position.x       = pt_center.x;
    pose.position.y       = pt_center.y;
    pose.position.z       = 0.5 * dyn_cld.h;
    pose.orientation.w    = 1.0;

    cylinder_mk.pose            = pose;
    cylinder_mk.scale.x = cylinder_mk.scale.y = dyn_cld.w;  // less then 1
    cylinder_mk.scale.z                       = dyn_cld.h;
    cylinders_vis.markers.push_back(cylinder_mk);
    cylinder_mk.id += 1;
  }

  cloud_all.width = cloud_all.points.size();
  cloud_all.height = 1;
  cloud_all.is_dense = true;

  // ROS_WARN_STREAM("Publishing " << cloud_all.points.size() << " points");
  // ROS_WARN_STREAM("Publishing " << cylinders.points.size() << " cylinders");
  // ROS_WARN_STREAM("Publishing " << cylinders_vis.markers.size() << " cylinders markers");

  // publish cloud
  pcl::toROSMsg(cloud_all, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(globalMap_pcd);

  // publish cylinder markers
  pcl::toROSMsg(cylinders, globalCylinders_pcd);
  globalCylinders_pcd.header.frame_id = _frame_id;
  _all_map_cylinder_pub.publish(globalCylinders_pcd);

  _all_map_cylinder_pub_vis.publish(cylinders_vis);
  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_map_sensing");
  ros::NodeHandle n("~");

  //_local_map_pub = n.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
  _all_map_cloud_pub    = n.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  _all_map_cylinder_pub = n.advertise<sensor_msgs::PointCloud2>("global_cylinders", 1);
  _all_map_cylinder_pub_vis =
      n.advertise<visualization_msgs::MarkerArray>("global_cylinders_vis", 1);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);

  // clearance for multi robots.
  _x_size -= 2.0;
  _y_size -= 2.0;

  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);
  n.param("map/frame_id", _frame_id, string("map"));

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);
  n.param("ObstacleShape/upper_vel", _v_h, 0.1);
  n.param("ObstacleShape/set_cylinder", _set_cylinder, false);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);

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

  ros::Duration(0.5).sleep();

  RandomMapGenerate();

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    // delete old cylinders in rviz
    visualization_msgs::Marker delete_cylinders;
    delete_cylinders.header.frame_id = _frame_id;
    delete_cylinders.action          = visualization_msgs::Marker::DELETEALL;
    cylinders_vis.markers.clear();
    cylinders_vis.markers.push_back(delete_cylinders);
    _all_map_cylinder_pub_vis.publish(cylinders_vis);

    // update map
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}