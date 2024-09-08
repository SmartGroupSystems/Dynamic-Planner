#include "bspline_race/bspline_race.h"

namespace FLAG_Race

{
    plan_manager::plan_manager(ros::NodeHandle &nh)
    {
        setParam(nh);
        TrajPlanning(nh);
    }
    plan_manager::~plan_manager() {}

    void plan_manager::setParam(ros::NodeHandle &nh)
    {
        last_time_ = ros::Time::now().toSec();
        nh.param("planning/traj_order", p_order_, 3);
        nh.param("planning/dimension", Dim_, -1);
        nh.param("planning/TrajSampleRate", TrajSampleRate, -1);
        nh.param("planning/max_vel", max_vel_, -1.0);
        nh.param("planning/max_acc", max_acc_, -1.0);
        nh.param("planning/goal_x", goal_x_, -1.0);
        nh.param("planning/goal_y", goal_y_, -1.0);
        nh.param("planning/lambda1",lambda1_,-1.0);
        nh.param("planning/lambda2",lambda2_,-1.0);
        nh.param("planning/lambda3",lambda3_,-1.0);
        nh.param("planning/frame",frame_,std::string("odom"));
        nh.param("planning/map_resolution",map_resolution_,-1.0);
        nh.param("planning/start_x",start_x_,-1.0);
        nh.param("planning/start_y",start_y_,-1.0);
        nh.param("planning/start_x",startPoint_x,-1.0);
        nh.param("planning/start_y",startPoint_y,-1.0);
        nh.param("planning/safe_distance",safe_distance_,-1.0);
        nh.param("planning/esdf_collision",esdf_collision,2.0);
        nh.param("planning/dist_p",dist_p,0.5);

        nh.param("gvo/vmap_x",vmap_x_,60);
        nh.param("gvo/vmap_y",vmap_y_,60);
        nh.param("gvo/v_x",v_x_,-3.0);
        nh.param("gvo/v_y",v_y_,3.0);
        nh.param("gvo/delta_t",delta_t_,0.03);
        nh.param("gvo/quad_r",quad_r_,0.3);
        nh.param("gvo/K1",K1_,1.0);
        nh.param("gvo/K2",K2_,3.0);        
        nh.param("gvo/K3",K3_,2.0);
        nh.param("gvo/v_safe",v_safe_,10.0);
        
        beta = max_vel_/dist_p;

        
        // sdf_map_.reset(new SDFMap);
        // sdf_map_->initMap(nh);
        // edt_environment_.reset(new EDTEnvironment);
        // edt_environment_->setMap(sdf_map_);
        map_size_x = (fabs(startPoint_x)+0.05)* 2 * (1/map_resolution_);
        map_size_y = (fabs(startPoint_y)+0.05)* 2 * (1/map_resolution_);
        esdf_map_ = 100* Eigen::MatrixXd::Ones(map_size_x,map_size_y);
        
        opt.reset(new bspline_optimizer);
        opt->setDimandOrder(Dim_,p_order_);
        opt->setOptParam(lambda1_,lambda2_,lambda3_,safe_distance_);
        opt->setVelAcc(max_vel_,max_acc_);
        opt->setMapParam(origin_x_,origin_y_,map_resolution_,start_x_,start_y_);
        // opt->setEnvironment(edt_environment_);
        
        u.reset(new UniformBspline);
        u->setOrderandBetaandDim(p_order_,beta,Dim_);

        gvo.reset(new Gvo(vmap_x_,vmap_y_,v_x_,v_y_,delta_t_,quad_r_));
        gvo->getParam(K1_,K2_,K3_,v_safe_);
    }

    void plan_manager::TrajPlanning(ros::NodeHandle &nh)
    {
        //goal_suber = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, & plan_manager::uav_goal_subCallback, this);
        fsm_suber = nh.subscribe<std_msgs::Int64>("/flag_detect",1,&plan_manager::fsm_subCallback,this);

        //订阅地图
        map_suber = nh.subscribe<std_msgs::Float64MultiArray>("/ESDFmsgs",1,&plan_manager::esdf_map_subCallback,this);
        
        //订阅路径
        path_suber = nh.subscribe<nav_msgs::Path>("/astar_node/grid_twist",1, &plan_manager::astar_subCallback,this);

        //订阅起止点
        // waypoint_suber = nh.subscribe<nav_msgs::Path>("/waypoint",1, &plan_manager::smooth_subCallback,this);

        //发布轨迹
        Traj_puber = nh.advertise<bspline_race::BsplineTraj>("/bspline_traj", 10);
        Time_puber = nh.advertise<std_msgs::Float64>("/back_time", 10);

        //订阅动态障碍物
        dynobs_suber = nh.subscribe<map_generator::dynamic_obs>("/dynamic_obs_local",1,&plan_manager::dynobs_callback,this);

        //发布轨迹转化为SO(3) command
        //Position_cmd = nh.advertise<bspline_race::PositionCommand>("position_cmd",50);

        //可视化执行的轨迹   
        Traj_vis = nh.advertise<nav_msgs::Path>("/traj_vis", 10);
        Traj_vis1 = nh.advertise<nav_msgs::Path>("/traj_smooth", 10);

        //可视化地图
        Map_puber = nh.advertise<visualization_msgs::Marker>("/esdfmap_slice", 10);
        Gvo_vis   = nh.advertise<visualization_msgs::Marker>("/gvo_slice", 10);
        Vogrid_vis   = nh.advertise<visualization_msgs::Marker>("/vogrid__slice", 10);
        optvel_puber = nh.advertise<visualization_msgs::Marker>("/opt_vel", 10);
        inivel_puber = nh.advertise<visualization_msgs::Marker>("/ini_vel", 10);
        guidevel_puber = nh.advertise<visualization_msgs::Marker>("/guide_vel", 10);
        guidepoints_puber = nh.advertise<visualization_msgs::Marker>("/guide_points", 10);
        nearguidepoints_puber = nh.advertise<visualization_msgs::Marker>("/nearguide_points", 10);
        traj_smooth = nh.advertise<bspline_race::BsplineTraj>("bspline_smooth",10);

        //同步飞机位置速度消息
        subscriber_vel = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh,"/mavros/local_position/velocity_local_orb",10);
        subscriber_pos = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh,"/mavros/local_position/pose_orb",10);
        subscriber_acc = new message_filters::Subscriber<sensor_msgs::Imu>(nh,"/mavros/imu/data",10);
        sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *subscriber_vel, *subscriber_pos, *subscriber_acc);  
        sync->registerCallback(boost::bind(&plan_manager::current_state_callback,this, _1, _2, _3));

        //获取当前aim
        // fullaim_suber = nh.subscribe<mavros_msgs::PositionTarget>("/mavbs/setpoint_raw/local",1,&plan_manager::fullaim_callback,this);
        arrived_suber = nh.subscribe<std_msgs::Int64>("/astar_node/target_arrived",10,&plan_manager::arrive_callback,this);

    }

    void plan_manager::arrive_callback(const std_msgs::Int64::ConstPtr & msg)
    {
        if(msg->data>0)
        {
            lambda3_ = lambda3_saved;
        }
        // cout<<lambda3_<<endl;
    }

    void plan_manager::fsm_subCallback(const std_msgs::Int64::ConstPtr & msg)
    {
        if(msg->data == 3)
            enable_flag = true;
        else 
            enable_flag = false;
    }

    void plan_manager::dynobs_callback(const map_generator::dynamic_obsConstPtr &msg)
    {
        if (msg->obs_num != 0)
        {
            dyn_obs_.header   = msg->header;
            dyn_obs_.obs_num  = msg->obs_num;
            dyn_obs_.position = msg->position;
            dyn_obs_.velocity = msg->velocity;
            dyn_obs_.radius   = msg->radius;
        
            gvo->getObsInfo(dyn_obs_);

            // cout << "gvo generation time is :  "<<time <<endl;
            get_dynobs_ = true;
            // ROS_INFO("DETECT DYNAMIC OBSTACLE.");
        }        

        double current_time = ros::Time::now().toSec();
        // cout << "curr time is :  "<<current_time <<endl;
        vel_slice_output(gvo->gvo_);
        vo_slice_output(gvo->vo_grid_);
        double after_time = ros::Time::now().toSec();
        // cout << "after time is :  "<<after_time <<endl;
        // double time = after_time-current_time;
        // cout << "gvo generation time is :  "<<time <<endl;

        guidepoints_output(gvo->guidepoints);
        nearguidepoints_output(gvo->near_guidepoints);
    }

    void plan_manager::current_state_callback(const geometry_msgs::TwistStampedConstPtr & vel_msg,
                                          const geometry_msgs::PoseStampedConstPtr &pos_msg,
                                          const sensor_msgs::ImuConstPtr &imu_msg)
    {
        // current_pos<< pos_msg->pose.position.x,pos_msg->pose.position.y;
        // current_vel<< vel_msg->twist.linear.x,vel_msg->twist.linear.y;
        // current_acc<< imu_msg->linear_acceleration.x,-imu_msg->linear_acceleration.y;// BUG!!
    }

    void plan_manager::uav_goal_subCallback(const geometry_msgs::PoseStampedConstPtr &goal_msg)
    {
        terminal_state(0,0) = goal_msg->pose.position.x;
        terminal_state(0,1) = goal_msg->pose.position.y; 
    }

    void plan_manager::esdf_map_subCallback(const std_msgs::Float64MultiArrayConstPtr &map_msg)
    {
        get_map = true;
        map_size_x = map_msg->data[0];
        map_size_y = map_msg->data[1];
        int index_x = map_msg->data[2];//min_cut(1)
        int index_y = map_msg->data[3];//min_cut(0)
        index_x = (fabs(startPoint_x)+0.05)* 2 * (1/map_resolution_)- index_x - map_size_x;

        // cout << "map_size_x  " << map_size_x << " map_size_y  " << map_size_y << endl;
        map_size = map_size_x * map_size_y;

        Eigen::MatrixXd grid_map_(map_size_x, map_size_y);
        Eigen::MatrixXd tmp_esdf(map_size_x, map_size_y);

        std::unique_ptr<double[]> src(new double[map_size]);
        std::unique_ptr<double[]> dst(new double[map_size]);

        for (size_t i = 0; i < map_size_y; i++)
        {
            for (size_t j = 0; j < map_size_x; j++)
            {
                grid_map_(map_size_x - j - 1, i) = map_msg->data[i * map_size_x + j + 4];
            }
        }

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                //把grid_map里的数据送入src指针
                *(src.get() + i * map_size_y + j) = grid_map_(i, j);
            }
        }
        // map_slice_output(grid_map_);

        computeEDT(dst.get(), src.get(), map_size_y, map_size_x);// map_size_y 图像宽度，map_size_x 图像高度

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                tmp_esdf(i, j) = static_cast<int>(sqrt(*(dst.get() + i * map_size_y + j)));
            }
        }

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                esdf_map_(index_x + i, index_y+j) = tmp_esdf(i, j);
            }
        }

        map_slice_output(esdf_map_);
        // cout<< "get esdf map"<<endl;

    }

    void plan_manager::astar_subCallback(const nav_msgs::PathConstPtr &path)
    { 
        // if(!get_map) return;
        // get_map = false;
        astar_path_.clear();
        // get_path = true;
        //读取Astar
        Eigen::Vector2d tmp_point;
        double delta_t = 0.1;
        Eigen::Vector2d current_aim = Eigen::Vector2d::Zero();
        Eigen::Vector2d current_vel = Eigen::Vector2d::Zero();
        Eigen::Vector2d current_acc = Eigen::Vector2d::Zero();
        current_vel << path->poses[0].pose.position.x,path->poses[0].pose.position.y;
        current_seq = path->poses[0].pose.position.z;
        current_acc << path->poses[1].pose.position.x,path->poses[1].pose.position.y;
        current_aim << path->poses[2].pose.position.x,path->poses[2].pose.position.y;
        for (size_t i = 2; i < path->poses.size(); i++)
        {
            tmp_point<< path->poses[i].pose.position.x,path->poses[i].pose.position.y;
            //A star路径是正的
            astar_path_.push_back(tmp_point);
        }
        //读取首末位置
        Eigen::Vector2d start_point,end_point;
        Eigen::Vector3d end_point_3d, end_point_3dlocal;
        start_point = *astar_path_.begin();
        end_point = *(astar_path_.end()-1);
        end_point_3d << end_point(0), end_point(1), 0.0;
        initial_state.resize(3,2);
        terminal_state.resize(3,2);
        std_msgs::Float64 msg_time;
        msg_time.data = back_time_;
        Time_puber.publish(msg_time);
        if(get_dynobs_)
        {   
            // cout << "catch "<< gvo->obs_num_ <<" dynamic obstacle" <<endl;
            get_dynobs_ = false;
            v_global_initial << current_vel(0),current_vel(1),0.0;
            fromGlobalToLocal(v_global_initial,v_local_intial);
            TransfromGlobalToLocal(end_point_3d,end_point_3dlocal);

            gvo->getTargetpoint(end_point_3dlocal);
            gvo->getLocalVel(v_local_intial);
            gvo->calcGuideVel();
            //  cout<< "point is:" << end_point_3d.transpose()<<endl;
            //  cout<< "local point is:" << end_point_3dlocal.transpose()<<endl;
            
            double current_time = ros::Time::now().toSec();
            gvo->optimizeVel();
            double after_time = ros::Time::now().toSec();
            double time = after_time-current_time;
            // cout << "comuptation time is :  "<<time <<endl;

            v_local_optimal = gvo->opt_vel_;
            //vis
            optvel_output(end_point_3dlocal,0);
            optvel_output(v_local_optimal,1);
            optvel_output(gvo->v_guide_,2);
            guidepoints_output(gvo->guidepoints);
            
            fromLocalToGlobal(v_local_optimal,v_global_optimal);
            
            initial_state <<    current_aim(0), current_aim(1),
                                v_global_optimal(0), v_global_optimal(1),
                                current_acc(0), current_acc(1);
            terminal_state <<   end_point(0), end_point(1),
                                0.0, 0.0,
                                0.0, 0.0;
        }
        
        else
        {
            initial_state <<    current_aim(0), current_aim(1),
                                current_vel(0), current_vel(1),
                                current_acc(0), current_acc(1);
            terminal_state <<   end_point(0), end_point(1),
                                0.0, 0.0,
                                0.0, 0.0;
        }

        double now_time_  = ros::Time::now().toSec() ;
        double delta_time = now_time_ - last_time_;//|| last_endpoint != end_point
        if( first_rifine == true || delta_time > 0.2 || checkTrajCollision() == true )// 
        {
            last_time_ = now_time_;
            first_rifine = false;
            last_endpoint = end_point;
            ros::Time time_1 = ros::Time::now();
            opt->setPath(astar_path_);
            opt->setEsdfMap(esdf_map_);
            u->setIniandTerandCpsnum(initial_state,terminal_state,opt->cps_num_);
            if(opt->cps_num_ == 2*p_order_)
            {
                // ROS_INFO("NO NEED OPT, CAUSE CONTROL POINTS NUM IS %d",opt->cps_num_);
                return;
            }
            UniformBspline spline = *u;
            opt->setSplineParam(spline);
            double current_time = ros::Time::now().toSec();
            opt->optimize();
            double after_time = ros::Time::now().toSec();
            double time = after_time-current_time;
            // cout << "traj generation time is :  "<<time <<endl;

            // printinfo();

            u->setControlPoints(opt->control_points_);
            u->getT(TrajSampleRate);
            UniformBspline p = *u;
            UniformBspline v = p.getDerivative();
            UniformBspline a = v.getDerivative();
            UniformBspline j = a.getDerivative();
            
            //生成轨迹
            geometry_msgs::PoseStamped tmp_p,tmp_v,tmp_a,tmp_j;
            geometry_msgs::PoseStamped tmp_vis;
            p_ = p.getTrajectory(p.time_);
            v_ = v.getTrajectory(p.time_);
            a_ = a.getTrajectory(p.time_);
            j_ = j.getTrajectory(p.time_);

            ros::Time time_2 = ros::Time::now();
            // ROS_WARN("[A*]{sucess}  Time in Traj_opt is %f ms", (time_2 - time_1).toSec() * 1000. );
            
            traj.position.clear();
            traj.velocity.clear();
            traj.acceleration.clear();
            traj.jerk.clear();
            traj_vis.poses.clear();   
               
            for (size_t i = 0; i < p_.rows(); i++)
            {
                // int count = p_.rows()-i-1;
                int count = i;
                tmp_p.header.seq = count;
                tmp_v.header.seq = count;
                tmp_a.header.seq = count;
                tmp_j.header.seq = count;

                tmp_p.pose.position.x   = p_(count,0);tmp_p.pose.position.y   = p_(count,1); tmp_p.pose.position.z   = 0;
                tmp_v.pose.position.x   = v_(count,0); tmp_v.pose.position.y  = v_(count,1); tmp_v.pose.position.z   = 0;
                tmp_a.pose.position.x   = a_(count,0); tmp_a.pose.position.y  = a_(count,1); tmp_a.pose.position.z   = 0; 
                tmp_j.pose.position.x   = j_(count,0); tmp_j.pose.position.y  = j_(count,1); tmp_j.pose.position.z   = 0;
                tmp_vis.pose.position.x = p_(count,0);tmp_vis.pose.position.y = p_(count,1);tmp_vis.pose.position.z = 1.0;
                traj.position.push_back(tmp_p) ;
                traj.velocity.push_back(tmp_v) ;
                traj.acceleration.push_back(tmp_a);
                traj.jerk.push_back(tmp_j);
                
                traj_vis.poses.push_back(tmp_vis);
                traj.header.frame_id = frame_;
                traj_vis.header.frame_id = frame_;
            }
            Traj_vis.publish(traj_vis);
            Traj_vis1.publish(traj_vis1);

            //发布期望轨迹
            traj.current_seq = current_seq;
            Traj_puber.publish(traj);
        }
            
    }

    void plan_manager::map_slice_output(const Eigen::MatrixXd &esdf_matrix)
    {
        visualization_msgs::Marker marker_result;
        marker_result.header.frame_id = "world";
        marker_result.type = visualization_msgs::Marker::POINTS;
        marker_result.action = visualization_msgs::Marker::MODIFY;
        marker_result.scale.x = 0.1;
        marker_result.scale.y = 0.1;
        marker_result.scale.z = 0.1;
        marker_result.pose.orientation.x = 0;
        marker_result.pose.orientation.y = 0;
        marker_result.pose.orientation.z = 0;
        marker_result.pose.orientation.w = 1;

        /* 遍历矩阵的所有元素 */
        for (int i = 0; i < esdf_matrix.rows(); i++)
        {
            for (int j = 0; j < esdf_matrix.cols(); j++)
            {
                double h = esdf_matrix(i, j);
                double max_dist = 20.0;
                if (h < -15.0 || h > 15.0) continue;
                /* 计算当前栅格中心的真实坐标 */
                double vox_pos_x, vox_pos_y;
                vox_pos_x = (j+0.5)*0.1 + (startPoint_x-0.05);
                vox_pos_y = (startPoint_y+0.05) - (i+0.5)*0.1;
                geometry_msgs::Point pt;
                pt.x = vox_pos_x;
                pt.y = vox_pos_y;
                pt.z = -0.3;
                marker_result.points.push_back(pt);

                /* 计算色彩 */
                std_msgs::ColorRGBA color;
                color.a = 1;
                std::vector<float> color_result;
                color_result = calculate_color(h, max_dist, -max_dist, R, G, B);
                color.r = color_result[0];
                color.g = color_result[1];
                color.b = color_result[2];
                marker_result.colors.push_back(color);
            }
        }
        Map_puber.publish(marker_result);
    }

    void plan_manager::vel_slice_output(const Eigen::MatrixXd &gvo)
    {
        visualization_msgs::Marker marker_result;
        marker_result.header.frame_id = "base";
        marker_result.type = visualization_msgs::Marker::POINTS;
        marker_result.action = visualization_msgs::Marker::MODIFY;
        marker_result.scale.x = 0.1;
        marker_result.scale.y = 0.1;
        marker_result.scale.z = 0.1;
        marker_result.pose.orientation.x = 0;
        marker_result.pose.orientation.y = 0;
        marker_result.pose.orientation.z = 0;
        marker_result.pose.orientation.w = 1;

        for (int i = 0; i < gvo.rows(); i++)
        {
            for (int j = 0; j < gvo.cols(); j++)
            {
                double h = gvo(i, j);
                double max_dist = 20.0;
                if (h < -15.0 || h > 15.0) continue;
                /* 计算当前栅格中心的真实坐标 */
                double vox_pos_x, vox_pos_y;
                vox_pos_x = (j+0.5)*0.1 + (v_x_-0.05);
                vox_pos_y = (v_y_+0.05) - (i+0.5)*0.1;
                geometry_msgs::Point pt;
                pt.x = vox_pos_x;
                pt.y = vox_pos_y;
                pt.z = -1.0;
                marker_result.points.push_back(pt);

                /* 计算色彩 */
                std_msgs::ColorRGBA color;
                color.a = 1;
                std::vector<float> color_result;
                color_result = calculate_color(h, max_dist, -max_dist, R, G, B);
                color.r = color_result[0];
                color.g = color_result[1];
                color.b = color_result[2];
                marker_result.colors.push_back(color);
            }
        }
        Gvo_vis.publish(marker_result);
    }

    void plan_manager::vo_slice_output(const Eigen::MatrixXd &vogrid)
    {
        visualization_msgs::Marker marker_result;
        marker_result.header.frame_id = "base";
        marker_result.type = visualization_msgs::Marker::POINTS;
        marker_result.action = visualization_msgs::Marker::MODIFY;
        marker_result.scale.x = 0.1;
        marker_result.scale.y = 0.1;
        marker_result.scale.z = 0.1;
        marker_result.pose.orientation.x = 0;
        marker_result.pose.orientation.y = 0;
        marker_result.pose.orientation.z = 0;
        marker_result.pose.orientation.w = 1;

        for (int i = 0; i < vogrid.rows(); i++)
        {
            for (int j = 0; j < vogrid.cols(); j++)
            {
                double h = vogrid(i, j);
                double max_dist = 20.0;
                if (h < -15.0 || h > 15.0) continue;
                /* 计算当前栅格中心的真实坐标 */
                double vox_pos_x, vox_pos_y;
                vox_pos_x = (j+0.5)*0.1 + (v_x_-0.05);
                vox_pos_y = (v_y_+0.05) - (i+0.5)*0.1;
                geometry_msgs::Point pt;
                pt.x = vox_pos_x;
                pt.y = vox_pos_y;
                pt.z = -1.0;
                marker_result.points.push_back(pt);

                /* 计算色彩 */
                std_msgs::ColorRGBA color;
                color.a = 1;
                std::vector<float> color_result;
                color_result = calculate_color(h, max_dist, -max_dist, R, G, B);
                color.r = color_result[0];
                color.g = color_result[1];
                color.b = color_result[2];
                marker_result.colors.push_back(color);
            }
        }
        Vogrid_vis.publish(marker_result);
    }

    std::vector<float>  plan_manager::calculate_color(double esdf_value, double max_dist, double min_dist, std::vector<int> R_values, std::vector<int> G_values, std::vector<int> B_values)
    {
        std::vector<float> color_result;

        /* 分段上色 */
        int colors_num = R_values.size();
        double dist_seg = (max_dist - min_dist) / colors_num;
        if (esdf_value > max_dist) esdf_value = max_dist;
        if (esdf_value < min_dist) esdf_value = min_dist;
        int seg_num = floor( (esdf_value - min_dist) / dist_seg );
        color_result.push_back((float)R_values[seg_num]/255.0);
        color_result.push_back((float)G_values[seg_num]/255.0);
        color_result.push_back((float)B_values[seg_num]/255.0);

        return color_result;
    }

    bool plan_manager::checkTrajCollision()
    {   
        traj_state state;//判断算出来的轨迹是否安全
        state = SAFE;
        Eigen::Vector2i tmp_index;//
        for (size_t i = 0; i < p_.rows(); i++)
        {
            tmp_index = posToIndex(p_.row(i));
            if(esdf_map_(tmp_index(0),tmp_index(1))<=esdf_collision)
            {
                // cout << "colision!"<<endl;
                state = COLLIDE;
                break;
            }
        }
        if(state == COLLIDE)
        {
            return true;//会撞
        } 
        else
            return false;//不会撞
    }

    void plan_manager::printinfo()
    {   
        cout <<"\033[46m--------------------grid_path - control_points_--------------------"<<endl;
        int i = 0;
        for(auto ptr : astar_path_)
        {Eigen::Vector2d xxx(opt->control_points_(3+i,0),opt->control_points_(3+i,1));
        cout << Eigen::Vector2d(ptr - xxx).norm() <<endl;
        i++;
        }
        cout <<"----------------------------------------\033[0m"<<endl;
        auto old_ptr = (*astar_path_.begin());
        cout <<"\033[45m--------------------grid_path--------------------"<<endl;
        for(auto ptr : astar_path_)
        {
        cout << ptr <<endl;
        cout << "- - -"<<endl;
        cout << Eigen::Vector2d(ptr - old_ptr).norm() <<endl;
        cout << "- - -"<<endl;
        old_ptr = ptr;
        }
        cout <<"----------------------------------------\033[0m"<<endl;
        cout <<"\033[45m--------------------control_points_--------------------"<<endl;
        i = 0;
        for(auto ptr : astar_path_)
        {Eigen::Vector2d xxx(opt->control_points_(3+i,0),opt->control_points_(3+i,1));
        cout << xxx <<endl;
        cout << "- - -"<<endl;
        i++;
        }
        cout <<"----------------------------------------\033[0m"<<endl;
    }

    Eigen::MatrixXd plan_manager::getSmoothTraj(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &end)
    {
        //目前该函数仅考虑二维平面的运动，三维运动将会尽快迭代
        Eigen::MatrixXd traj;
        initial_state.resize(3,2);
        terminal_state.resize(3,2);
        initial_state <<start.pose.position.x, start.pose.position.y,
                                1.0, 0.0,
                                3.0, 0.0;
        terminal_state<< end.pose.position.x, end.pose.position.y,
                                0.0, 0.0,
                                0.0, 0.0;
        double dist = sqrt(pow(end.pose.position.x-start.pose.position.x,2)+
                                    pow(end.pose.position.y-start.pose.position.y,2));
            
        opt.reset(new bspline_optimizer(Dim_,p_order_,dist));//只考虑smooth的构造
        u.reset(new UniformBspline(p_order_,opt->cps_num_,beta,Dim_,initial_state,terminal_state));
        opt->setSmoothParam(lambda1_,lambda2_,max_vel_,max_acc_);
        opt->initialControlPoints(*u);
        opt->optimizesmooth();            //计算轨迹

        u->setControlPoints(opt->control_points_);
        u->getT(TrajSampleRate);
        UniformBspline p = *u;
        UniformBspline v = p.getDerivative();
        UniformBspline a = v.getDerivative();
        p_ = p.getTrajectory(p.time_);
        v_ = v.getTrajectory(p.time_);
        a_ = a.getTrajectory(p.time_);
        traj.resize(p_.rows(),p_.cols()*3);
        //traj : px py vx vy ax ay
        for (size_t i = 0; i < traj.rows(); i++)
        {    
            traj(i,0)= p_(i,0);
            traj(i,1)= p_(i,1); 
            traj(i,2)= v_(i,0);
            traj(i,3)= v_(i,1);
            traj(i,4)= a_(i,0);
            traj(i,5)= a_(i,1);
        }   
        return traj;
    }

    void plan_manager::optvel_output(const Eigen::Vector3d &opt_vel, const int type)
    {       
        optvel.header.frame_id = "base";
        optvel.header.stamp = ros::Time::now();
        optvel.ns = "optvel";
        optvel.action = visualization_msgs::Marker::ADD;
        optvel.id = 0;
        optvel.type = visualization_msgs::Marker::ARROW;
        optvel.scale.x = 0.1;
        optvel.scale.y = 0.2;
        optvel.scale.z = 0.0;
        optvel.color.a = 2;
        optvel.color.g = 0.0;
        optvel.color.b = 1.0;
        geometry_msgs::Point p;//start and end point
        p.x = 0; p.y = 0; p.z = 0.0;
        optvel.points.push_back(p);
        p.x= opt_vel(0); p.y = opt_vel(1); p.z = 0.0;
        optvel.points.push_back(p);
        switch (type)
        {
            case 0:
                optvel.type = visualization_msgs::Marker::POINTS;
                optvel.color.r = 0.3;
                inivel_puber.publish(optvel);
                break;
            case 1:
                optvel.color.r = 0.5;
                optvel_puber.publish(optvel); 
                break;
            case 2:
                optvel.color.r = 1.0;
                guidevel_puber.publish(optvel);
                break;
            default:
                break;
        }
        optvel.points.clear();     
    }

    void plan_manager::guidepoints_output(const std::vector<Eigen::Vector2d> &points)
    {   
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base";
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        for (int i = 0; i < points.size(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = points[i](0);
            pt.y = points[i](1);
            pt.z = 0.0;
            marker.points.push_back(pt);
        }
        guidepoints_puber.publish(marker);
    }

    void plan_manager::nearguidepoints_output(const std::vector<Eigen::Vector2d> &points)
    {   
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base";
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 2;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        for (int i = 0; i < points.size(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = points[i](0);
            pt.y = points[i](1);
            pt.z = 0.0;
            marker.points.push_back(pt);
        }
        nearguidepoints_puber.publish(marker);
    }
    void plan_manager::fromGlobalToLocal(const Eigen::Vector3d v_global, Eigen::Vector3d &v_local)
    {
        tf::StampedTransform globalToLocalTransform;
        listener.lookupTransform("base", "world", ros::Time(0), globalToLocalTransform);
        tf::Vector3 v_g(v_global(0),v_global(1),v_global(2));
        tf::Matrix3x3 rotation(globalToLocalTransform.getRotation());
        tf::Vector3 v_l = rotation * v_g;
        v_local << v_l.x(), v_l.y(), 0.0;
        
        // cout<< "v_global is: "<< v_global.transpose()<<endl;
        // cout<< "v_local is: " << v_local.transpose() <<endl;
    }

    void plan_manager::fromLocalToGlobal(const Eigen::Vector3d v_local, Eigen::Vector3d &v_global)
    {
        tf::StampedTransform localToGlobalTransform;
        listener.lookupTransform("world", "base", ros::Time(0), localToGlobalTransform);
        tf::Vector3 v_l(v_local(0),v_local(1),v_local(2));
        tf::Matrix3x3 rotation(localToGlobalTransform.getRotation());
        tf::Vector3 v_g = rotation * v_l;
        v_global << v_g.x(), v_g.y(), 0.0; 

        // cout<< "v_global is: "<< v_global.transpose()<<endl;
        // cout<< "v_local is: " << v_local.transpose() <<endl;
    }

    void plan_manager::TransfromGlobalToLocal(const Eigen::Vector3d p_global, Eigen::Vector3d &p_local)
    {
        tf::StampedTransform globalToLocalTransform;
        listener.lookupTransform("base", "world", ros::Time(0), globalToLocalTransform);
        tf::Vector3 tmp_global;
        tmp_global.setX(p_global(0));
        tmp_global.setY(p_global(1));
        tmp_global.setZ(p_global(2));
        tf::Vector3 tmp_local;
        tmp_local = globalToLocalTransform * tmp_global;
        p_local(0) = tmp_local.x();
        p_local(1) = tmp_local.y();
        p_local(2) = 0.0;
    }
}