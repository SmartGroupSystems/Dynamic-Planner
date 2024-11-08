#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>
#include <map_generator/moving_cylinder.h>
#include <map_generator/moving_circle.h>
#include <map_generator/dynamic_obs.h>

using namespace std;

void dynCylinder(const Eigen::Vector2d &point);
void staticwall(const Eigen::Vector2d &point);
void RandomDynMapGenerate();

ros::Publisher all_map_pub_;
ros::Publisher dynamic_obs_pub_;
sensor_msgs::PointCloud2 map_msg_;
pcl::PointCloud<pcl::PointXYZ> map_cloud_, _static_cloud_ ,_dyn_cloud_;

vector<Eigen::Vector3d> points_,dyn_points_;

std::vector<dynamic_map_objects::MovingCylinder> _dyn_cylinders;
std::vector<dynamic_map_objects::MovingCircle>   _dyn_circles;

int         _obs_num, _static_obs_num;
double      _obs_traj_length, _static_obs_length;
double      _x_size, _y_size, _z_size;
double      w_l,h_l,v_h,resolution,_rate, len2_;;
int         _dyn_cylinder_num = 0;

/**@brief map mode
 * 0: randomize both vx and vy
 * 1: randomize vx, vy = 0
 * 2: randomize vy, vx = 0
 */
int _mode;

void RandomDynMapGenerate()
{
  //1.
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(-fabs(_x_size/2), fabs(_x_size/2));//map is square
  std::vector<Eigen::Vector2d> candidatepoints; 
  
  std::random_device ad;
  std::mt19937 aen(ad());
  std::uniform_real_distribution<double> ang(0.0, 2.0 * M_PI);
  std::vector<Eigen::Vector2d> candidateuniformvectors; 

  for (int i = 0; i < _obs_num; ++i) 
  {
    //mid point generation
    Eigen::Vector2d point(dist(gen),dist(gen));
    candidatepoints.push_back(point);
    //uniform angle generation
    Eigen::Vector2d angle(std::cos(ang(aen)),std::sin(ang(aen)));
    candidateuniformvectors.push_back(angle);
  }

  for (int i = 0; i < _obs_num; ++i) 
  {
    Eigen::Vector2d p1(candidatepoints[i](0)- _obs_traj_length/2 * candidateuniformvectors[i](0),
                       candidatepoints[i](1)- _obs_traj_length/2 * candidateuniformvectors[i](1));
    Eigen::Vector2d p2(candidatepoints[i](0)+ _obs_traj_length/2 * candidateuniformvectors[i](0),
                       candidatepoints[i](1)+ _obs_traj_length/2 * candidateuniformvectors[i](1));
    dynCylinder(p1);
    dynCylinder(p2);
  }
    
  //这里需要补充随机生成静态障碍物的代码
}

void staticwall(const Eigen::Vector2d &point)
{
  double x = point(0);
  double y = point(1);
  points_.push_back(Eigen::Vector3d(x, y, 0));
  if (points_.size() < 2) return;

  // Generate wall using two points
  Eigen::Vector3d p1 = points_[0];
  Eigen::Vector3d p2 = points_[1];
  points_.clear();

  Eigen::Vector3d dir1 = (p2 - p1).normalized();
  double len = (p2 - p1).norm();
  Eigen::Vector3d dir2;
  dir2[0] = -dir1[1];
  dir2[1] = dir1[0];

  pcl::PointXYZ pt_random;
  for (double l1 = 0.0; l1 <= len + 1e-3; l1 += 0.1) {
    Eigen::Vector3d tmp1 = p1 + l1 * dir1;
    for (double l2 = -len2_; l2 <= len2_ + 1e-3; l2 += 0.1) {
      Eigen::Vector3d tmp2 = tmp1 + l2 * dir2;
      for (double h = -0.5; h < 2.5; h += 0.1) {
        pt_random.x = tmp2[0];
        pt_random.y = tmp2[1];
        pt_random.z = h;
        _static_cloud_.push_back(pt_random);
      }
    }
  }
}

void dynCylinder(const Eigen::Vector2d &point) {
  double x = point(0);
  double y = point(1);
  
  dyn_points_.push_back(Eigen::Vector3d(x, y, 0));
  if (dyn_points_.size() < 2) return;
  
  //Generate moving cylinders between two given points...
  Eigen::Vector3d p1 = dyn_points_[0];
  Eigen::Vector3d p2 = dyn_points_[1];
  dyn_points_.clear();
  //cout<<"dist is : "<< (p2-p1).norm()<<endl;
  dynamic_map_objects::MovingCylinder cylinder(p1,p2,w_l,h_l,v_h,resolution);
  cylinder.setMode(_mode);
  _dyn_cylinders.push_back(cylinder);
  for (size_t i = 0; i < cylinder._cloud.size(); i++)
  { 
    _dyn_cloud_.push_back(cylinder._cloud[i]);
  }
  _dyn_cylinder_num ++; 
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "click_map");
  ros::NodeHandle n("~");

  n.param("map/len2", len2_, 0.15);
  n.param("map/w_l", w_l, 1.0);
  n.param("map/h_l", h_l, 4.5);
  n.param("map/v_h", v_h, 0.05);// v =  v_h * rate
  n.param("map/resolution", resolution, 0.1);
  n.param("map/mode", _mode, 0);
  n.param("map/rate", _rate, 20.0);

  n.param("map/x_size",  _x_size, 40.0);
  n.param("map/y_size",  _y_size, 40.0);
  n.param("map/z_size",  _z_size, 5.0);
  n.param("map/dyn_obs_num", _obs_num, 10);
  n.param("map/dyn_obs_traj", _obs_traj_length, 5.0);
  n.param("map/static_obs_num", _static_obs_num, 10);
  n.param("map/static_obs_length", _static_obs_length, 5.0);

  //clear point cloud
  _dyn_cloud_.clear();
  _static_cloud_.clear();
  _dyn_cylinders.clear();
  _dyn_circles.clear();
  map_cloud_.clear();

  all_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/click_map", 10);
  dynamic_obs_pub_ =
      n.advertise<map_generator::dynamic_obs>("/dynamic_obs",10);
 
  RandomDynMapGenerate();

  ros::Rate loop_rate(_rate);
  int header_stamp = 0;
  while (ros::ok()) 
  {
    map_generator::dynamic_obs dyn_obs_msg_;
    for (size_t i = 0; i < _dyn_cylinder_num; i++)
    {
      _dyn_cylinders[i].updateclick();
      for (size_t j = 0; j < _dyn_cylinders[i]._cloud.size(); j++)
       {
        _dyn_cloud_.push_back(_dyn_cylinders[i]._cloud[j]);
       }
      geometry_msgs::Vector3 pos;
      geometry_msgs::Vector3 vel; 
      pos.x = _dyn_cylinders[i].x;
      pos.y = _dyn_cylinders[i].y;
      pos.z = 0.0;
      vel.x = _dyn_cylinders[i].vx * _rate;
      vel.y = _dyn_cylinders[i].vy * _rate;
      vel.z = 0.0;
      dyn_obs_msg_.position.push_back(pos);
      dyn_obs_msg_.velocity.push_back(vel);
      dyn_obs_msg_.radius.push_back(_dyn_cylinders[i].w/2);
    }

    //update map
    map_cloud_ = _static_cloud_ + _dyn_cloud_;
    map_cloud_.width = map_cloud_.points.size();
    map_cloud_.height = 1;
    map_cloud_.is_dense = true;
  
    //pubmap
    pcl::toROSMsg(map_cloud_, map_msg_);
    map_msg_.header.frame_id = "world";
    //cout<< "map cloud size is : "<< map_cloud_.size()<< endl;
    all_map_pub_.publish(map_msg_);

    //pub dynamic_obstacle
    dyn_obs_msg_.header.seq = header_stamp;
    dyn_obs_msg_.header.frame_id = "world";
    dyn_obs_msg_.obs_num = _dyn_cylinder_num;
    dynamic_obs_pub_.publish(dyn_obs_msg_);

    //clear and refresh
    map_cloud_.clear();
    _dyn_cloud_.clear();
    header_stamp ++ ;

    //loop for dynamic
    ros::spinOnce();
    loop_rate.sleep();
  }
}