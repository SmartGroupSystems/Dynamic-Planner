#include "bspline_race/bspline_race.h"

using namespace std;
// bspline
bool first_bs = true;
#define PI acos(-1)
#define INF 999.9
bool arrived = false;
#define T_RATE 50.0
double set_height = 0.8;//这个参数后期可以修改
double roll, pitch, yaw;//定义存储r\p\y的容器
double last_yaw;
double last_yaw_dot;
int connect_seq = 0;
Eigen::Vector3d vel_drone_fcu;
static geometry_msgs::PoseStamped aim;
bspline_race::PositionCommand pva_msg;
nav_msgs::Path vis_path;
class bs_traj
{
    public:
        Eigen::Vector3d pos_;
        Eigen::Vector3d vel_;
        Eigen::Vector3d acc_;
        Eigen::Vector3d jerk_;
        int seq_;
};
std::vector<bs_traj> BTraj;
geometry_msgs::PoseStamped debug_msg;
int current_pub_seq = 0;
double delta_T = 0.02;
double D_YAW_MAX = PI/2;
double output_yaw;
double output_d_yaw;
double YAW_MAX = D_YAW_MAX * delta_T;
std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw);
std::pair<double, double> cal_yaw( double current_yaw,double aim_yaw);

ros::Publisher cmd_pub,
               bs_pub,
               debug_pub,
               vis_path_pub;
ros::Subscriber pts_sub,
                cmd_sub,
                pos_sub;

void run()
{
  if(BTraj.size() != 0)
        {
            bs_traj BT_ptr = *(BTraj.begin());
            if(BT_ptr.seq_ == connect_seq || BT_ptr.seq_ == (connect_seq+1))
            {
              debug_msg.pose.position.x = BT_ptr.vel_[0];
              debug_msg.pose.position.y = BT_ptr.vel_[1];
              debug_msg.pose.orientation.x = BT_ptr.acc_[0];
              debug_msg.pose.orientation.y = BT_ptr.acc_[1];
            }
            debug_msg.pose.orientation.w = sqrt(pow(BT_ptr.vel_[0],2.0) +  pow(BT_ptr.vel_[1],2.0));
            BTraj.erase(BTraj.begin());

            // CAL_YAW
            double arg_    = atan2(-BT_ptr.vel_[0],BT_ptr.vel_[1]) + (PI/2.0f);
            double vel_len = sqrt(pow(BT_ptr.vel_[0],2)+pow(BT_ptr.vel_[1],2));
            if(vel_len<=0.1) arg_ = last_yaw;
            std::pair<double, double> yaw_all = calculate_yaw(last_yaw,arg_);
            geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(yaw_all.first);

        /*   AIM   This is subscibed by quadrotor!*/
            geometry_msgs::PoseStamped pos_ptr;
            // POSE
            pos_ptr.pose.position.x = BT_ptr.pos_[0];
            pos_ptr.pose.position.y = BT_ptr.pos_[1];
            pos_ptr.pose.position.z = set_height;
            
            aim.pose.position = pos_ptr.pose.position;
            // YAW
            aim.pose.orientation = geo_q;
            
            // POSE
            pva_msg.position.x = BT_ptr.pos_[0];
            pva_msg.position.y = BT_ptr.pos_[1];
            pva_msg.position.z = set_height;
            // VEL
            pva_msg.velocity.x = BT_ptr.vel_[0];
            pva_msg.velocity.y = BT_ptr.vel_[1];
            pva_msg.velocity.z = 0;
            // ACC
            pva_msg.acceleration.x = BT_ptr.acc_[0];
            pva_msg.acceleration.y = BT_ptr.acc_[1];
            pva_msg.acceleration.z = 0;
            //JERK
            pva_msg.jerk.x = BT_ptr.jerk_[0];
            pva_msg.jerk.y = BT_ptr.jerk_[1];
            pva_msg.jerk.z = 0;

            // YAW
            // pva_msg.yaw      = yaw_all.first;
            pva_msg.yaw      = 0.0;
            pva_msg.yaw_dot  = yaw_all.second;


        /*   BS    This is subscibed by Astar! */
            mavros_msgs::PositionTarget bs_msg;
            int seq_interval = T_RATE*2/5;
            int last_traj_seq      = (*(BTraj.end()-1)).seq_;
            int first_traj_seq     = (*(BTraj.begin())).seq_;
            int remain_traj_length = last_traj_seq- first_traj_seq;

            int to_bs_seq = (remain_traj_length>seq_interval) ? first_traj_seq + seq_interval : last_traj_seq;
            int to_bs_seq_index = to_bs_seq - first_traj_seq;
            if(BTraj.size()<=to_bs_seq_index) return;
            BT_ptr = BTraj[to_bs_seq_index];
            // POSE
            bs_msg.position.x = BT_ptr.pos_[0];
            bs_msg.position.y = BT_ptr.pos_[1];
            bs_msg.position.z = set_height;
            // VEL
            bs_msg.velocity.x = BT_ptr.vel_[0];
            bs_msg.velocity.y = BT_ptr.vel_[1];
            bs_msg.velocity.z = 0;
            // ACC
            bs_msg.acceleration_or_force.x = BT_ptr.acc_[0];
            bs_msg.acceleration_or_force.y = BT_ptr.acc_[1];
            bs_msg.acceleration_or_force.z = 0;

            bs_msg.yaw = (float)(to_bs_seq);
            bs_pub.publish(bs_msg);
            /*   PUB_CONTROL   */

            // TIME
            pva_msg.header.stamp = ros::Time::now();
            cmd_pub.publish(pva_msg);
            debug_pub.publish(debug_msg);
            //cout<< pva_msg<<endl;
            // VIS
            geometry_msgs::PoseStamped vis_msg;
            vis_msg.pose.position = pva_msg.position;
            vis_path.poses.push_back(vis_msg);
            vis_path_pub.publish(vis_path);
        }
        else
        {
            // cout <<"[cmd] Arrived!"<< endl;
        }

}

void pose_subCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{

}
void traj_cb(const bspline_race::BsplineTrajConstPtr &msg)
{
  // 收到新轨迹
  if(first_bs || BTraj.empty())
  {
    // 直接放到BTraj里
    for (size_t i = 0; i < msg->position.size(); i++)
    {    
        bs_traj BT_ptr;
        BT_ptr.pos_ << msg->position[i].pose.position.x, 
                       msg->position[i].pose.position.y,
                       msg->position[i].pose.position.z;
        BT_ptr.vel_ << msg->velocity[i].pose.position.x, 
                       msg->velocity[i].pose.position.y,
                       msg->velocity[i].pose.position.z;
        BT_ptr.acc_ << msg->acceleration[i].pose.position.x, 
                       msg->acceleration[i].pose.position.y,
                       msg->acceleration[i].pose.position.z;
        BT_ptr.jerk_<< msg->jerk[i].pose.position.x,
                       msg->jerk[i].pose.position.y,
                       msg->jerk[i].pose.position.z;       
        BT_ptr.seq_ = i;
        BTraj.push_back(BT_ptr);
    }
    first_bs = false;
  }
  else
  {
    // 轨迹拼接
    int new_traj_start_seq = msg->current_seq ;
    // cout<<"current sequnce is:"<< new_traj_start_seq << endl;
    int delta_seq = new_traj_start_seq - BTraj[0].seq_;
    // cout << "delta_seq: " << delta_seq <<endl;
    if(delta_seq<0)// 如果当前发过来的轨迹太旧
    {
      ROS_WARN("The delay is too large < %i > points away, < %f > ms ago, check the parameter Settings."
                ,-delta_seq, -delta_seq*delta_T*1000);
        ROS_ERROR("USLESS.");
    }
    else if(delta_seq>BTraj.size())// 如果这个新轨迹的起点超过了当前剩余曲线的终点
    {
      ROS_WARN("The scope of replanning is too large, check the parameter Settings.");
      ROS_ERROR("Wait, something really big has happened...");
    }
    else// 终于没事了
    {
      std::vector<bs_traj> BTraj_remain;
      auto start_ptr_ = BTraj.begin();
      // auto end_ptr_   = BTraj.begin() + delta_seq;// BUG ?悬垂指针？
      int add_seq_;
      if(delta_seq < (BTraj.size()-1))  
      {         
          add_seq_  = delta_seq-1;
        //   cout << "delta_seq\n"<<delta_seq<<endl;
      }
      else
      {
        cout << "啊？"<<endl;
        add_seq_ = (BTraj.size()-1);
      }
      auto end_ptr_   = BTraj.begin() + add_seq_;
      if(add_seq_ > 0)
      BTraj_remain.assign(start_ptr_,end_ptr_);// 存储剩余的可用路径
      int old_traj_end_seq = end_ptr_->seq_;
      (*(BTraj_remain.end()-1)).seq_;
      for (size_t i = 0; i < msg->position.size(); i++)
      {    
          bs_traj BT_ptr;
            BT_ptr.pos_ << msg->position[i].pose.position.x, 
                       msg->position[i].pose.position.y,
                       msg->position[i].pose.position.z;
            BT_ptr.vel_ << msg->velocity[i].pose.position.x, 
                       msg->velocity[i].pose.position.y,
                       msg->velocity[i].pose.position.z;
            BT_ptr.acc_ << msg->acceleration[i].pose.position.x, 
                       msg->acceleration[i].pose.position.y,
                       msg->acceleration[i].pose.position.z;
            BT_ptr.jerk_<< msg->jerk[i].pose.position.x,
                       msg->jerk[i].pose.position.y,
                       msg->jerk[i].pose.position.z;   
          BT_ptr.seq_ = i + new_traj_start_seq;
          BTraj_remain.push_back(BT_ptr);
      }
      BTraj = BTraj_remain;
      // cout << "Successfully connect the trajectory at < "<<old_traj_end_seq
      //                                      <<" > ++++ < "<<new_traj_start_seq<<" >."<<endl;
                
    }
  }
  // ROS_INFO("New seq begin at: < %i >, end at: < %i >.",(*(BTraj.begin())).seq_,(*(BTraj.end()-1)).seq_);
}
void bspline_subCallback(const bspline_race::BsplineTrajConstPtr &msg)
{
  bool affine_traj = true;
  std::vector<bs_traj> BTraj_saved = BTraj;
    int remain_last_seq = 0;
    if(!first_bs)//&& BTraj.size() != 0
    {
      // 軌跡拼接
        int  to_bs_seq = msg->current_seq;
        cout <<"to_bs_seq: "<< to_bs_seq <<endl;
        while ((*(BTraj.end()-1)).seq_ >= to_bs_seq)
        {
            BTraj.erase(BTraj.end()-1);
            if(BTraj.size() == 0)
            {
                ROS_ERROR("STOP BECAUSE OF NEW TRAJ IS USELESS!!!");
                affine_traj = false;
                break;
            }
        }
        remain_last_seq = to_bs_seq;
        
        if(affine_traj) 
        {
          connect_seq = (*(BTraj.end()-1)).seq_;
          ROS_INFO("Successfully connect the trajectory at < %i > ++++ < %i >. With vel: %f, %f acc: %f, %f",
                  (*(BTraj.end()-1)).seq_,remain_last_seq,
                  (*(BTraj.end()-1)).vel_[0],(*(BTraj.end()-1)).vel_[1],
                  (*(BTraj.end()-1)).acc_[0],(*(BTraj.end()-1)).acc_[1]);
        }
        cout <<(*(BTraj.end()-1)).pos_<<endl;
        cout<<"--------------"<<endl;
        cout <<msg->position[0].pose.position.x<<"\n"<< msg->position[0].pose.position.y <<endl;
    }
    first_bs = false;
    if(!affine_traj)
    {
      BTraj = BTraj_saved;
      ROS_WARN("Use saved traj. Seq begin at: < %i >, end at: < %i >.",(*(BTraj.begin())).seq_,(*(BTraj.end()-1)).seq_);
      return;
    }
    for (size_t i = 0; i < msg->position.size(); i++)
    {    
        bs_traj BT_ptr;
        BT_ptr.pos_ << msg->position[i].pose.position.x    , msg->position[i].pose.position.y     , msg->position[i].pose.position.z;
        BT_ptr.vel_ << msg->velocity[i].pose.position.x    , msg->velocity[i].pose.position.y     , msg->velocity[i].pose.position.z;
        BT_ptr.acc_ << msg->acceleration[i].pose.position.x, msg->acceleration[i].pose.position.y , msg->acceleration[i].pose.position.z,
        BT_ptr.jerk_<< msg->jerk[i].pose.position.x        , msg->jerk[i].pose.position.y         , msg->jerk[i].pose.position.z;
        BT_ptr.seq_ = i + remain_last_seq;
        BTraj.push_back(BT_ptr);
    }
    ROS_INFO("Seq begin at: < %i >, end at: < %i >.",(*(BTraj.begin())).seq_,(*(BTraj.end()-1)).seq_);

    // for(auto iter : BTraj)
    // {
    //   cout << "\033[46m----  ----"<<endl;
    //   cout << iter.vel_ <<endl;
    // }
    // cout << "\033[0m" << endl;

}
void rcvWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr & wp)
{     
  first_bs = true;
  BTraj.clear();
  vis_path.poses.clear();
  vis_path.header.frame_id = "world";
}

std::pair<double, double> cal_yaw( double current_yaw,double aim_yaw)
{

  std::pair<double, double> yaw_yawdot(0, 0);
  if(current_yaw<0)                 current_yaw = current_yaw + 2*PI;
  else if(current_yaw>2*PI)  current_yaw = current_yaw - 2*PI;
    if(aim_yaw<0)                 aim_yaw = aim_yaw + 2*PI;
  else if(aim_yaw>2*PI)    aim_yaw = aim_yaw - 2*PI;
  double yaw_distance = aim_yaw - current_yaw;
  double sign_        = yaw_distance / fabs(yaw_distance);
  if(fabs(yaw_distance) < YAW_MAX )
  {cout<<"ca1"<<endl;
    output_yaw   = aim_yaw;
    output_d_yaw = yaw_distance / delta_T;
  }
  else
  {cout<<"ca2"<<endl;
    output_yaw = current_yaw + sign_ * YAW_MAX;
    output_d_yaw = sign_*D_YAW_MAX;
  }
  yaw_yawdot.first = output_yaw;
  yaw_yawdot.second = output_d_yaw;
  return yaw_yawdot;
}
std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw)
{
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw_ = 0;
  double yawdot = 0;
  if (aim_yaw - current_yaw > PI)
  {
    
    if (aim_yaw - current_yaw - 2 * PI < -YAW_MAX)
    {
      yaw_ = current_yaw - YAW_MAX;
      if (yaw_ < -PI)
        yaw_ += 2 * PI;

      yawdot = -D_YAW_MAX;
    }
    else
    {
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw > PI)
        yawdot = -D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
  else if (aim_yaw - current_yaw < -PI)
  {
    if (aim_yaw - current_yaw + 2 * PI > YAW_MAX)
    {
      yaw_ = current_yaw + YAW_MAX;
      if (yaw_ > PI)
        yaw_ -= 2 * PI;

      yawdot = D_YAW_MAX;
    }
    else
    {
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw < -PI)
        yawdot = D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
  else
  {
    if (aim_yaw - current_yaw < -YAW_MAX)
    {
      yaw_ = current_yaw - YAW_MAX;
      if (yaw_ < -PI)
        yaw_ += 2 * PI;

      yawdot = -D_YAW_MAX;
    }
    else if (aim_yaw - current_yaw > YAW_MAX)
    {
      yaw_ = current_yaw + YAW_MAX;
      if (yaw_ > PI)
        yaw_ -= 2 * PI;

      yawdot = D_YAW_MAX;
    }
    else
    {
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw > PI)
        yawdot = -D_YAW_MAX;
      else if (yaw_ - current_yaw < -PI)
        yawdot = D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
    if (fabs(yaw_ - last_yaw) <= YAW_MAX)
    yaw = 0.5 * last_yaw + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot + 0.5 * yawdot;
  last_yaw = yaw_;  
  last_yaw_dot = yawdot;
  yaw_yawdot.first = yaw_;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle nh("~");
    cmd_pub      = nh.advertise<bspline_race::PositionCommand>("/position_cmd", 10);
    bs_pub       = nh.advertise<mavros_msgs::PositionTarget>("/mavbs/setpoint_raw/local" , 1);
    debug_pub    = nh.advertise<geometry_msgs::PoseStamped>("/debug",1);
    vis_path_pub = nh.advertise<nav_msgs::Path>("/pubed_path",1);
    
    pts_sub   = nh.subscribe<geometry_msgs::PoseStamped>( "/move_base_simple/goal", 10, &rcvWaypointsCallback );
    cmd_sub   = nh.subscribe<bspline_race::BsplineTraj>("/bspline_traj",1, &traj_cb);
    pos_sub   = nh.subscribe<geometry_msgs::PoseStamped>("/odom_visualization/pose",1,&pose_subCallback);
    ros::Rate rate(T_RATE);
while(ros::ok())
    {
      run();
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}