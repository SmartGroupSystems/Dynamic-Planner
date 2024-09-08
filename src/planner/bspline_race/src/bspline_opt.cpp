#include <bspline_race/bspline_opt.h>

namespace FLAG_Race

{
    bspline_optimizer::bspline_optimizer(const std::vector<Eigen::Vector2d> &path, const int&Dim,  const int&p)
    {
        path_.clear();
        path_ = path;
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = path.size() + 2*p_order_ -2;
    }

    bspline_optimizer::bspline_optimizer(const int&Dim,  const int&p, const double &dist)
    {
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = 2*p_order_+floor(dist/1.0);
    }
    
    bspline_optimizer::~bspline_optimizer(){}
    
    void bspline_optimizer::setOptParam(const double lambda1,const double lambda2,const double lambda3,
                                                                                    const double safe_dist)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            lambda3_ = lambda3;
            safe_distance_ = safe_dist;
    }

    void bspline_optimizer::setVelAcc(const double vel, const double acc)
    {
            max_vel_ = vel;
            max_acc_ = acc;
    }

    // void bspline_optimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
    //     this->edt_environment_ = env;
    // }

    void bspline_optimizer::setSmoothParam(const double lambda1,const double lambda2,
                                                            const double vel, const double acc)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            max_vel_ = vel;
            max_acc_ = acc;
    }

    void bspline_optimizer::setEsdfMap(const Eigen::MatrixXd &esdf_map)
    {    
        esdf_map_ = esdf_map;
    }

    void bspline_optimizer::setMapParam(const double &origin_x,const double &origin_y, const double &map_resolution,
                                                                                    const double &start_x, const double &start_y)
    {
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        map_resolution_ = map_resolution;
        startPoint_x = start_x;
        startPoint_y = start_y;
    }

    void bspline_optimizer::setSplineParam(const UniformBspline &u)
    {
        u_ = u;
        bspline_interval_  = u.interval_;
        beta_ = u.beta_;
        control_points_.resize(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u_.getBoundConstraintb();
        
        for (size_t i = 0; i < Dim_; i++)
        {
                for (size_t j = 0; j < p_order_; j++)
                {
                     control_points_(j,i) = beq_bound(i*cps_num_+j);
                     control_points_((1)*cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);
                }
        }
            
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
            control_points_.row(i+p_order_) = path_[i+1];

        }
    }

    void bspline_optimizer::initialControlPoints(UniformBspline u)
    {
        control_points_.setZero(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u.getBoundConstraintb();
        for (size_t i = 0; i < Dim_; i++)
        {
        for (size_t j = 0; j < p_order_; j++)
        {
        control_points_(j,i) = beq_bound(i*cps_num_+j);
        control_points_(cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);
        }
        }
        int insert_num = cps_num_-2*p_order_;
        Eigen::Vector2d start_pos = control_points_.row(p_order_-1);
        Eigen::Vector2d end_pos = control_points_.row(cps_num_-p_order_);
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
        control_points_(i+p_order_,0) = start_pos(0)+(end_pos(0)-start_pos(0))/(insert_num+1)*(i+1) ;
        control_points_(i+p_order_,1) = start_pos(1)+(end_pos(1)-start_pos(1))/(insert_num+1)*(i+1) ;
        }
       // cout<<control_points_<<endl;
    }

    void bspline_optimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
    {
        cost = 0.0;
        if (falg_use_jerk)
        {
            Eigen::Vector2d jerk, temp_j;

            for (int i = 0; i < q.cols() - 3; i++)
            {
                /* evaluate jerk */
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                temp_j = 2.0 * jerk;
                /* jerk gradient */
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        }
        else
        {
            Eigen::Vector2d acc, temp_acc;

            for (int i = 0; i < q.cols() - 2; i++)
            {
                /* evaluate acc */
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                /* acc gradient */
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }

    void bspline_optimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
    {
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector2d vi = (q.col(i + 1) - q.col(i)) / ts;
      for (int j = 0; j < 2; j++)
      {
        if (vi(j) > max_vel_)
        {
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }
    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector2d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 2; j++)
      {
        if (ai(j) > max_acc_)
        {
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }

    }
    }
    
    void bspline_optimizer::calcEsdfCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
    {
        cost = 0.0;
        double  dist;
        Eigen::Vector2d dist_grad;
        Eigen::Vector3d dist_grad_3d;
        Eigen::Vector3d q_3d;

        for (int i = p_order_; i < q.cols()-p_order_; i++) 
        {
            dist = calcDistance(q.col(i));
            dist_grad = calcGrad(q.col(i));
            
            // q_3d << q(0,i), q(1,i) , 0.8; 
            // edt_environment_->evaluateEDTWithGrad(q_3d, -1.0, dist, dist_grad_3d);
            // cout << "dist_grad_3d is "<< dist_grad_3d.transpose()<<endl;
            // dist_grad << dist_grad_3d(0), dist_grad_3d(1);

            if (dist_grad.norm() > 1e-4) dist_grad.normalize();
            if (dist < safe_distance_) 
            {
                cost += pow(dist - safe_distance_, 2);
                gradient.col(i) += 2.0 * (dist - safe_distance_) * dist_grad;     
            }
        }   
    }

    double bspline_optimizer::calcDistance(const Eigen::MatrixXd &q)
    {
        double dist;
        Eigen::Vector2d p(q(0,0),q(1,0));//存入两个控制点
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];//4个邻居点
        getSurroundPts(p,sur_pts, diff);
        double dists[2][2];
        getSurroundDistance(sur_pts, dists);
        interpolateBilinearDist(dists, diff, dist);
        return dist;
    }

    void bspline_optimizer::interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff, 
                                                                                                                                                                double& dist)
    {
        //二线性插值
        double c00 = values[0][0];
        double c01 = values[0][1];
        double c10 = values[1][0];
        double c11 = values[1][1];
        double tx = diff(0);
        double ty = diff(1);
        
        double nx0 = lerp(c00,c10,ty);
        double nx1 = lerp(c01,c11,ty);
        double ny0 = lerp(c00,c01,tx);
        double ny1 = lerp(c10,c11,tx);

        dist = lerp(ny0,ny1,ty);
    }
    Eigen::Vector2d bspline_optimizer::calcGrad(const Eigen::MatrixXd &q)
    {
        Eigen::Vector2d p(q(0,0),q(1,0));//存入两个控制点
        Eigen::Vector2d dist_grad;
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];//4个邻居点
        getSurroundPts(p,sur_pts, diff);

        double dists[2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateBilinear(dists, diff, dist_grad);

        return dist_grad;
    }

    void bspline_optimizer::getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff)
    {
        double dist_x = pos(0) - startPoint_x;
        double dist_y = startPoint_y - pos(1);
        diff(0) = fmod(dist_x,map_resolution_);
        diff(1) = fmod(dist_y,map_resolution_);

        Eigen::Vector2d curr_index;//用这个索引找到最左上角的点，并记为 p(0,0);
        curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < 2; j++)
            {       
                Eigen::Vector2d tmp_index(curr_index(0)+i,curr_index(1)+j);
                pts[i][j] = tmp_index;
            }
        }
    }
    void bspline_optimizer::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
    {
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < 2; j++)
            {
              Eigen::Vector2d tmp_index = pts[i][j];
              dists[i][j] = esdf_map_(tmp_index(0),tmp_index(1));  
            }
        }
    }

    void bspline_optimizer::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad)
    {
        //二线性插值
        double c00 = values[0][0];
        double c01 = values[0][1];
        double c10 = values[1][0];
        double c11 = values[1][1];
        double tx = diff(0);
        double ty = diff(1);
        
        double nx0 = lerp(c00,c10,ty);
        double nx1 = lerp(c01,c11,ty);
        double ny0 = lerp(c00,c01,tx);
        double ny1 = lerp(c10,c11,tx);

        grad(0) = (nx1- nx0)/map_resolution_;
        grad(1) = (ny0- ny1)/map_resolution_;
    }

    void bspline_optimizer::combineCost( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine)
    {
        Eigen::MatrixXd control_points = control_points_.transpose();
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points(i,j+p_order_) = x[j*Dim_+i];
                } 
            }
        f_combine = 0.0;
        Eigen::MatrixXd grad2D; 
        grad2D.resize(Dim_,cps_num_);
        grad2D.setZero(Dim_,cps_num_);//初始化梯度矩阵

        double f_smoothness, f_length, f_distance, f_feasibility;
        Eigen::MatrixXd g_smoothness_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_feasibility_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_distance_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        f_smoothness  = f_feasibility =  f_distance = 0.0;
        // cout<< control_points.transpose()<<endl;
        calcSmoothnessCost(control_points, f_smoothness, g_smoothness_);
    // cout<<"====================calcSmoothnessCost"<<endl;
        calcFeasibilityCost(control_points,f_feasibility,g_feasibility_);
    // cout<<"====================calcFeasibilityCost"<<endl;
        calcEsdfCost(control_points,f_distance,g_distance_);
    // cout<<"====================calcEsdfCost"<<endl;

        f_combine = lambda1_ * f_smoothness + lambda2_*f_feasibility + lambda3_*f_distance;
        grad2D = lambda1_*g_smoothness_ + lambda2_ * g_feasibility_ +lambda3_ * g_distance_ ;
        grad = grad2D.block(0,p_order_,Dim_,cps_num_-2*p_order_);//起点  块大小
    }
    void bspline_optimizer::combineCostSmooth( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine)
    {
        Eigen::MatrixXd control_points = control_points_.transpose();
        //cout<<control_points<<endl;
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points(i,j+p_order_) = x[j*Dim_+i];
                } 
            }
        f_combine = 0.0;
        Eigen::MatrixXd grad2D; 
        grad2D.resize(Dim_,cps_num_);
        grad2D.setZero(Dim_,cps_num_);//初始化梯度矩阵

        double f_smoothness, f_feasibility;
        Eigen::MatrixXd g_smoothness_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        f_smoothness  =  0.0;
        calcSmoothnessCost(control_points, f_smoothness, g_smoothness_);
        grad2D = lambda1_*g_smoothness_ ;
        grad = grad2D.block(0,p_order_,Dim_,cps_num_-2*p_order_);//起点  块大小
    }

    double bspline_optimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                                                                         void* func_data)
    {
        bspline_optimizer* opt = reinterpret_cast<bspline_optimizer*>(func_data);
        Eigen::MatrixXd grad_matrix;
        double cost;
        opt->combineCost(x,grad_matrix,cost);
        opt->iter_num_++;

        for (size_t i = 0; i < grad_matrix.cols(); i++)
            {
                for (size_t j = 0; j <opt->Dim_; j++)
                {
                    // grad.push_back(grad_matrix(j,i)) ;
                    grad[i*opt->Dim_+j] = grad_matrix(j,i);
                }    
            } 
        /* save the min cost result */
        if (cost < opt->min_cost_) {
                opt->min_cost_     = cost;
                opt->best_variable_ = x;
            }
        return cost;
    }

    double bspline_optimizer::costFunctionSmooth(const std::vector<double>& x, std::vector<double>& grad,
                                                                                         void* func_data)
    {
        bspline_optimizer* opt = reinterpret_cast<bspline_optimizer*>(func_data);
        Eigen::MatrixXd grad_matrix;
        double cost;
        opt->combineCostSmooth(x,grad_matrix,cost);
        opt->iter_num_++;

        for (size_t i = 0; i < grad_matrix.cols(); i++)
            {
                for (size_t j = 0; j <opt->Dim_; j++)
                {
                    // grad.push_back(grad_matrix(j,i)) ;
                    grad[i*opt->Dim_+j] = grad_matrix(j,i);
                }    
            } 
        /* save the min cost result */
        if (cost < opt->min_cost_) {
                opt->min_cost_     = cost;
                opt->best_variable_ = x;
            }
        return cost;
    }


    void bspline_optimizer::optimize()
    {
            /* initialize solver */
            // cout << "/* initialize solver */"<<endl;
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num); 
            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
        {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
        {
            for (size_t i = 0; i < Dim_; i++)
            {
                control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
            } 
        }
        // cout<< "optimize successfully~"<<endl;
            // cout << "iner:\n"<<control_points_<<endl;
            // cout<<"iter num :"<<iter_num_<<endl;
    }
    void bspline_optimizer::optimizesmooth()
    {
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunctionSmooth,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 5.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
         {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

          for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
                } 
            }
            cout<< "optimize successfully~"<<endl;
    }

    void bspline_optimizer::optimize_withoutesdf()
    {
           double intial_lambda3 = lambda3_;
            lambda3_  = 0;
            /* initialize solver */
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
         {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

          for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
                } 
            }
            lambda3_  = intial_lambda3;
    }

}