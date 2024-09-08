#ifndef _DYNAMIC_LOCAL_H
#define _DYNAMIC_LOCAL_H

#include <map_generator/dynamic_obs.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

double sensing_range_;
double rate_;
map_generator::dynamic_obs dynamic_obs_;
geometry_msgs::PoseStamped curr_pose;

ros::Publisher  dynobslocal_pub;
ros::Subscriber dynobs_sub_;
ros::Subscriber pose_sub;
tf::StampedTransform globalToLocalTransform;
tf::TransformListener* listener;

bool isInSensingRange(const geometry_msgs::Vector3 &obs_pose);
geometry_msgs::Vector3 fromGlobalToLocal(const geometry_msgs::Vector3 gloal_info, const int type);
void dynObsCallback(const map_generator::dynamic_obsConstPtr &msg);
void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

#endif;
