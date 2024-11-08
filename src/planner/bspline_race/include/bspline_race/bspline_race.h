#ifndef  _BSPLINE_RACE_H
#define  _BSPLINE_RACE_H

//ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Imu.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
// #include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
//ros多线程
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>

//自定义
#include <bspline_race/UniformBspline.h>
#include <bspline_race/bspline_opt.h>
#include <bspline_race/BsplineTraj.h>
#include <bspline_race/PositionCommand.h>
#include <map_generator/dynamic_obs.h>
#include <bspline_race/gvo.h>
// #include <plan_env/edt_environment.h>

using namespace std;

namespace FLAG_Race
{
    class plan_manager
    {
        public:
            //从launch读取的参数
            double last_time_;
            double back_time_ ;
            int p_order_;// order of bspline
            int N_;// number of control points
            int Dim_;// dimension of traj
            int TrajSampleRate;// 轨迹采样频率
            double beta;
            double dist_p;//控制点之间的距离
            double max_vel_,max_acc_;//最大速度，加速度
            Eigen::MatrixXd initial_state,terminal_state;//初始，结束P V A
            double start_x_, start_y_;// A star找到的起点
            double goal_x_,goal_y_;//A star找到的终点
            int map_size_x,map_size_y,map_size;//esdf x y
            Eigen::MatrixXd esdf_map_;//esdf地图
            Eigen::MatrixXd grid_map_;//grid map
            std::string frame_;
            double map_resolution_;//地图分辨率
            double origin_x_, origin_y_;
            double startPoint_x,startPoint_y;//找到地图左上角的点，x负，y正
            double safe_distance_;//安全距离
            double esdf_collision;
            Eigen::MatrixXd p_,v_,a_,j_;//轨迹buffer

            int current_seq = 0;
            Eigen::Vector2d current_pos;
            Eigen::Vector2d last_endpoint;
            // Eigen::Vector2d current_aim = Eigen::Vector2d::Zero();
            // Eigen::Vector2d current_vel = Eigen::Vector2d::Zero();
            // Eigen::Vector2d current_acc = Eigen::Vector2d::Zero();
            //同步器
            typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, 
                                                                    geometry_msgs::PoseStamped, 
                                                                    sensor_msgs::Imu> syncPolicy;
            message_filters::Subscriber<geometry_msgs::TwistStamped>* subscriber_vel;
            message_filters::Subscriber<geometry_msgs::PoseStamped>* subscriber_pos;
            message_filters::Subscriber<sensor_msgs::Imu>* subscriber_acc;
            message_filters::Synchronizer<syncPolicy>* sync;

            //状态相关
            enum traj_state 
            {
                COLLIDE = 0,
                SAFE = 1
            };  //碰撞状态

            bool get_path = false;
            bool get_map = false;
            bool get_traj = false;
            bool first_rifine = true;
            bool enable_flag = false;

            //智能类指针
            std::shared_ptr<UniformBspline> u;
            std::shared_ptr<UniformBspline> u1;
            std::shared_ptr<bspline_optimizer> opt;
            std::shared_ptr<bspline_optimizer> opt1;
            std::shared_ptr<Gvo> gvo;
            // std::shared_ptr<EDTEnvironment> edt_environment_;
            // std::shared_ptr<SDFMap> sdf_map_;
            // EDTEnvironment::Ptr edt_environment_;
            // SDFMap::Ptr sdf_map_;

            //nlopt 相关
            double lambda1_,lambda2_,lambda3_,lambda3_saved;

            //gvo
            int vmap_x_,vmap_y_;// vel_grid_ size
            double v_x_,v_y_;// vel_grid_ start point
            double delta_t_;
            double quad_r_;
            double K1_,K2_,K3_;
            double v_safe_;
            bool get_dynobs_ = false;
            Eigen::Vector3d v_global_initial, v_global_optimal;
            Eigen::Vector3d v_local_intial, v_local_optimal;

            //ros
            ros::Subscriber goal_suber;//订阅RVIZ导航点，备用
            ros::Subscriber path_suber;//订阅A star路径点
            ros::Subscriber map_suber;//订阅esdf map
            ros::Publisher Traj_vis;//轨迹可视化发布
            ros::Publisher Traj_vis1;//轨迹可视化发布
            ros::Publisher Traj_puber;//发布轨迹
            //ros::Publisher Position_cmd;//SO(3)轨迹发布
            ros::Publisher Time_puber;
            ros::Publisher Map_puber;//发布esdf地图可视化
            ros::Publisher Gvo_vis;//
            ros::Publisher Vogrid_vis;//
            ros::Publisher traj_smooth;
            ros::ServiceServer fsm_call;
            ros::Subscriber state_suber;
            ros::Subscriber aim_suber;
            ros::Subscriber fullaim_suber;
            ros::Subscriber waypoint_suber;
            ros::Subscriber fsm_suber;
            ros::Subscriber arrived_suber;
            ros::Subscriber dynobs_suber;
            ros::Publisher optvel_puber;
            ros::Publisher inivel_puber;
            ros::Publisher guidevel_puber;
            ros::Publisher guidepoints_puber;
            ros::Publisher nearguidepoints_puber;
            tf::TransformListener listener;

            //ros msg
            nav_msgs::Path traj_vis;//轨迹可视化
            nav_msgs::Path traj_vis_;//轨迹可视化
            nav_msgs::Path traj_vis1;//轨迹可视化
            bspline_race::BsplineTraj traj;//执行轨迹
            bspline_race::BsplineTraj traj_;//执行轨迹
            bspline_race::PositionCommand cmd_;//so(3)轨迹指令
            std::vector<Eigen::Vector2d> astar_path_;
            map_generator::dynamic_obs dyn_obs_;
            visualization_msgs::Marker optvel;

            /* 色表 */
            vector<int> R = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
            240, 220, 200, 180, 160, 140, 120, 100, 80, 60, 40, 20, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240 };
            vector<int> G = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240,
            255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
            255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
            vector<int> B = { 0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240,
            255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
            255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
            240, 220, 200, 180, 160, 140, 120, 100, 80, 60, 40, 20, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        public:
            plan_manager(){}
            plan_manager(ros::NodeHandle &nh);
            ~plan_manager();
            void setParam(ros::NodeHandle &nh);//从ros节点中读取参数
            void TrajPlanning(ros::NodeHandle &nh);//轨迹规划
            bool checkTrajCollision();//检测轨迹是否发生碰撞
            void uav_goal_subCallback(const geometry_msgs::PoseStampedConstPtr &goal_msg);
            void esdf_map_subCallback(const std_msgs::Float64MultiArrayConstPtr &map_msg);
            std::vector<float>  calculate_color(double esdf_value, double max_dist, double min_dist, std::vector<int> R_values, std::vector<int> G_values, std::vector<int> B_values);
            void astar_subCallback(const nav_msgs::PathConstPtr &path);
            void map_slice_output(const Eigen::MatrixXd &esdf_matrix);
            void vel_slice_output(const Eigen::MatrixXd &gvo);
            void vo_slice_output(const Eigen::MatrixXd &vogrid);
            void optvel_output(const Eigen::Vector3d &opt_vel,const int type);
            void guidepoints_output(const std::vector<Eigen::Vector2d> &points);
            void nearguidepoints_output(const std::vector<Eigen::Vector2d> &points);
            bool fsm_callback(std_srvs::Trigger::Request & req,std_srvs::Trigger::Response & res);
            void current_state_callback(const geometry_msgs::TwistStampedConstPtr & vel_msg,
                                        const geometry_msgs::PoseStampedConstPtr &pos_msg,
                                        const sensor_msgs::ImuConstPtr &imu_msg);
            void aim_callback(const geometry_msgs::PoseStamped::ConstPtr & aim_msg);
            void fullaim_callback(const mavros_msgs::PositionTarget::ConstPtr & aim_msg);
            void smooth_subCallback(const nav_msgs::Path::ConstPtr & msg);
            void fsm_subCallback(const std_msgs::Int64::ConstPtr & msg);
            void arrive_callback(const std_msgs::Int64::ConstPtr & msg);
            void dynobs_callback(const map_generator::dynamic_obsConstPtr &msg);
            Eigen::MatrixXd getSmoothTraj(const geometry_msgs::PoseStamped &start,
                                          const geometry_msgs::PoseStamped &end);

            Eigen::Vector2i posToIndex(const Eigen::MatrixXd &pos)
            {
                Eigen::Vector2i curr_index;
                double dist_x = pos(0,0) - startPoint_x;
                double dist_y = startPoint_y - pos(0,1);
                curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
                return curr_index;
            }

            void fromGlobalToLocal(const Eigen::Vector3d v_global, Eigen::Vector3d &v_local);
            void fromLocalToGlobal(const Eigen::Vector3d v_local,  Eigen::Vector3d &v_global);
            void TransfromGlobalToLocal(const Eigen::Vector3d p_global, Eigen::Vector3d &p_local);
            
            //打印消息
            void printinfo();
    };

}
#endif
