#include <mapping/dynamic_local.h>

void dynObsCallback(const map_generator::dynamic_obsConstPtr &msg)
{
    if (msg->obs_num != 0)
    {
        dynamic_obs_.header = msg->header;
        dynamic_obs_.obs_num = msg->obs_num;
        dynamic_obs_.position = msg->position;
        dynamic_obs_.velocity = msg->velocity;
        dynamic_obs_.radius = msg->radius;
    }
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{  
    map_generator::dynamic_obs dynamic_obs_local_;
    curr_pose.pose.position.x = msg->pose.position.x;
    curr_pose.pose.position.y = msg->pose.position.y;
    curr_pose.pose.position.z = msg->pose.position.z;
    curr_pose.pose.orientation.w = msg->pose.orientation.w;
    curr_pose.pose.orientation.x = msg->pose.orientation.x;
    curr_pose.pose.orientation.y = msg->pose.orientation.y;
    curr_pose.pose.orientation.z = msg->pose.orientation.z;

    for (size_t i = 0; i < dynamic_obs_.obs_num; i++)
    {
        if(isInSensingRange(dynamic_obs_.position[i]))
        {
            dynamic_obs_local_.obs_num ++;
            dynamic_obs_local_.header.frame_id = "base";
            geometry_msgs::Vector3 local_pos,local_vel;
            local_pos = fromGlobalToLocal(dynamic_obs_.position[i],0);//pos needs translation and rotation
            local_vel  = fromGlobalToLocal(dynamic_obs_.velocity[i],1);//vel only needs rotation
            dynamic_obs_local_.position.push_back(local_pos);
            dynamic_obs_local_.velocity.push_back(local_vel);
            dynamic_obs_local_.radius.push_back(dynamic_obs_.radius[i]);
        }
    }

    if (dynamic_obs_local_.obs_num != 0)
    {
        dynobslocal_pub.publish(dynamic_obs_local_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_obs_local");
    ros::NodeHandle n("~");
    n.param("sensing_range", sensing_range_, 3.0);
    n.param("rate", rate_, 30.0);

    ros::Rate loop_rate(rate_);

    while (!ros::isInitialized()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    listener = new tf::TransformListener();

    dynobs_sub_ = n.subscribe("/dynamic_obs", 10, dynObsCallback);
    pose_sub    = n.subscribe("/odom_visualization/pose", 10, poseCallback);
    dynobslocal_pub = n.advertise<map_generator::dynamic_obs>("/dynamic_obs_local",10);
    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

bool isInSensingRange(const geometry_msgs::Vector3 &obs_pose)
{
    geometry_msgs::Vector3 quad_pose;
    quad_pose.x = curr_pose.pose.position.x;
    quad_pose.y = curr_pose.pose.position.y;
    quad_pose.z = curr_pose.pose.position.z;
    
    double dist = std::sqrt(pow(quad_pose.x-obs_pose.x,2) + std::pow(quad_pose.y-obs_pose.y,2));
    if (dist < sensing_range_)
        return true;
    else
        return false;
}

geometry_msgs::Vector3 fromGlobalToLocal(const geometry_msgs::Vector3 gloal_info, const int type)
{
    listener->lookupTransform("base", "world", ros::Time(0), globalToLocalTransform);
    tf::Matrix3x3 rotation(globalToLocalTransform.getRotation());
    tf::Vector3 tmp_global;
    tmp_global.setX(gloal_info.x);
    tmp_global.setY(gloal_info.y);
    tmp_global.setZ(gloal_info.z);
    tf::Vector3 tmp_local;
    if(type == 0)
    {
        tmp_local = globalToLocalTransform * tmp_global;
    }

    if(type == 1)
    {
        tmp_local = rotation * tmp_global;
    }

    geometry_msgs::Vector3 local_info;
    local_info.x = tmp_local.x();
    local_info.y = tmp_local.y();
    local_info.z = 0.0;
    return local_info;
}