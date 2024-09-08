#ifndef _DETECT_DYNAMIC_H
#define _DETECT_DYNAMIC_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vrpn_client_ros/vrpn_client_ros.h>
#include <string.h>
#include <iostream>
#include <map_generator/dynamic_obs.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/TransformStamped.h>


ros::Subscriber myPose_suber;
ros::Publisher  dynobslocal_pub;
tf::StampedTransform globalToLocalTransform;
tf::TransformListener* listener;
tf::TransformBroadcaster* broadcaster;

double sensing_range_;
double rate_;
double dynobs_radius_;
bool first_loop = true;
geometry_msgs::PoseStamped myPose;

ros::Time last_time_;
std::vector<geometry_msgs::Vector3> last_position_(3);
std::vector<geometry_msgs::Vector3> curr_position_(3);
std::vector<geometry_msgs::Vector3> curr_vel_(3);
void vrpnCallback(const geometry_msgs::PoseStampedConstPtr& pose1,
                  const geometry_msgs::PoseStampedConstPtr& pose2,
                  const geometry_msgs::PoseStampedConstPtr& pose3);
bool isInSensingRange(const geometry_msgs::Vector3 &obs_pose);
geometry_msgs::Vector3 fromGlobalToLocal(const geometry_msgs::Vector3 gloal_info, const int type);

#endif;