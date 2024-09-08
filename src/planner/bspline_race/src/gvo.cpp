#include "bspline_race/gvo.h"

/*
process:

note: the poses and vels of obstacles are in local frame
      make sure the quad vel to be transformed into local frame too!
Initialize
  |
  |     Gvo()
  |     getParam()
  |
Process
  |   
  |   getObsInfo()
  |     |
  |     | setVO()
  |     | setVoGrid()
  |     | setGvo()
  |     | calcGuideVel()
  |     |
  |    getLocalVel()
  |     |
  |    optimize()
  |
*/

namespace FLAG_Race
{
  Gvo::Gvo(const int &rows, const int &cols, const double &x,
           const double &y,const double &delta_t,const double &r)
  {
    map_x_ = rows;
    map_y_ = cols;
    x_ = x;
    y_ = y;
    delta_t_ = delta_t;
    quad_r_ = r;
    map_resolution_ = 2*fabs(x_)/map_x_;
    vo_grid_.setZero(map_x_,map_y_);
    gvo_.resize(map_x_,map_y_);
  }

  void Gvo::getObsInfo(const map_generator::dynamic_obs &info)//make sure the pose and vel are in local frame!
  {
    obs_pos_.clear();
    obs_vel_.clear();
    obs_size_.clear();
    obs_num_ = info.obs_num;
    for (size_t i = 0; i < obs_num_; i++)
    {
      obs_size_.push_back(info.radius[i]);
      Eigen::Vector3d tmp_pos(info.position[i].x,info.position[i].y,info.position[i].z);
      obs_pos_.push_back(tmp_pos);
      Eigen::Vector3d tmp_vel(info.velocity[i].x,info.velocity[i].y,info.velocity[i].z);
      obs_vel_.push_back(tmp_vel);
    }

    vo_.clear();
    vo_grid_.setZero(map_x_,map_y_);
    setVo();
    setVoGrid();
    setGvo();
    // calcGuideVel();
  }

  void Gvo::getLocalVel(const Eigen::Vector3d &v)
  {
    v_local_ = v;
  }

  void Gvo::getParam(const double K1,const double K2,const double K3,const double v_safe)
  {
    K1_ = K1;
    K2_ = K2;
    K3_ = K3;
    v_safe_ = v_safe;
  }

  void Gvo::setVo()
  {
    for (size_t i = 0; i < obs_num_ ; i++)
    {
      double r  =  obs_size_[i] + quad_r_;
      double y  =  obs_pos_[i](1)/obs_pos_[i](0);
      double y1 = -obs_pos_[i](0)/obs_pos_[i](1);
      double k1,k2;
      if((pow(obs_pos_[i](0),2)*pow(r,2)+pow(obs_pos_[i](1),2)*pow(r,2)- pow(r,4))<0)
      {
        cout<<"VO is complex,,, the result is not reliable"<<endl;
      }
      else
      {
        k1 = (-obs_pos_[i](0)*obs_pos_[i](1) + sqrt( pow(obs_pos_[i](0),2)*pow(r,2)+pow(obs_pos_[i](1),2)*pow(r,2)- pow(r,4) )) /
                (pow(r,2)-pow(obs_pos_[i](0),2));
        k2 = (-obs_pos_[i](0)*obs_pos_[i](1) - sqrt( pow(obs_pos_[i](0),2)*pow(r,2)+pow(obs_pos_[i](1),2)*pow(r,2)- pow(r,4) )) /
                (pow(r,2)-pow(obs_pos_[i](0),2));  

        double b1, b2;
        double b11, b12,b21,b22;
        b11 = r /(delta_t_)*sqrt(pow(y1,2)+1) - y1*obs_pos_[i](0)/delta_t_ + obs_pos_[i](1)/delta_t_;
        b12 = -r/(delta_t_)*sqrt(pow(y1,2)+1) - y1*obs_pos_[i](0)/delta_t_ + obs_pos_[i](1)/delta_t_;
        
        if(fabs(b11)>fabs(b12))
            b1 = b12;
        else
            b1 = b11;     
        
        double t = delta_t_*0.2;
        b21 =  r/t * sqrt(pow(y1,2)+1) - y1*obs_pos_[i](0)/t + obs_pos_[i](1)/t;
        b22 = -r/t * sqrt(pow(y1,2)+1) - y1*obs_pos_[i](0)/t + obs_pos_[i](1)/t;

        if(fabs(b21)>fabs(b22))
            b2 = b21;
        else
            b2 = b22;       
        
        Eigen::Vector3d tmp1,tmp2,tmp3,tmp4; 
        Eigen::MatrixXd vo;
        vo.resize(4,3);
        // vo.resize(3,3);

        if(k1*obs_pos_[i](0)-obs_pos_[i](1)>0)
          tmp1 << -k1,  1, 0;
        else
          tmp1 <<  k1, -1, 0;

        if(k2*obs_pos_[i](0)-obs_pos_[i](1)>0)
          tmp2 << -k2,  1, 0;
        else
          tmp2 <<  k2, -1, 0;

        if(y1*obs_pos_[i](0)-obs_pos_[i](1)+b1>0)
          tmp3 << -y1,  1,  b1;
        else
          tmp3 <<  y1, -1, -b1;

        if(y1*obs_pos_[i](0)-obs_pos_[i](1)+b2>0)
          tmp4 << -y1,  1,  b2;
        else
          tmp4 <<  y1, -1, -b2;   

        //VO = Mincowsky sum（obs_vel,vo） 
        //original: ax + by ≤ b ,  v = （c, d）
        //          x' = x + c & y' = y + d
        //          a(x' - c) + b(y' - d) ≤ b  ---> ax' + by' ≤ ac + bd + b
        //final:    ax + by ≤ ac + bd + b
        Eigen::Vector3d tmp1_,tmp2_,tmp3_,tmp4_;
        tmp1_ << tmp1(0), tmp1(1), tmp1(2) + tmp1(0)*obs_vel_[i](0) + tmp1(1)*obs_vel_[i](1);
        tmp2_ << tmp2(0), tmp2(1), tmp2(2) + tmp2(0)*obs_vel_[i](0) + tmp2(1)*obs_vel_[i](1);
        tmp3_ << tmp3(0), tmp3(1), tmp3(2) + tmp3(0)*obs_vel_[i](0) + tmp3(1)*obs_vel_[i](1);
        tmp4_ << tmp4(0), tmp4(1), tmp4(2) + tmp4(0)*obs_vel_[i](0) + tmp4(1)*obs_vel_[i](1);

        vo.row(0) = tmp1_;             
        vo.row(1) = tmp2_;
        vo.row(2) = tmp4_;
        vo.row(3) = tmp3_; 
        // cout << vo <<endl;
        // cout << "::::::::::::::::::::::::::::::::::::::::::" <<endl;
        vo_.push_back(vo);
      }  
    }
  }

  void Gvo::setVoGrid()
  {
    Eigen::Vector3d current_pt;
    for (size_t i = 0; i < vo_grid_.cols(); i++)
    {
      for (size_t j = 0; j< vo_grid_.rows(); j++)
      {
        current_pt << x_ + i*map_resolution_, y_ - j*map_resolution_, 0.0;
        if(getState(current_pt))
          vo_grid_(j,i) = 1;
      }    
    } 
    // cout << vo_grid_ << endl;   
    // cout << "::::::::::::::::::::::::::::::::::::::::::" <<endl;
  }

  void Gvo::setGvo()
  {
    double *src_v,*dst_v, *src1_v, *dst1_v;
    const int map_size = gvo_.rows() * gvo_.cols();

    src_v  = (double*)malloc(map_size*sizeof(double));
    dst_v  = (double*)malloc(map_size*sizeof(double));
    src1_v = (double*)malloc(map_size*sizeof(double));
    dst1_v = (double*)malloc(map_size*sizeof(double));

    for (size_t i = 0; i < gvo_.rows(); i++)
    {
      for (size_t j = 0; j < gvo_.cols(); j++)
      {
        *(src_v  + i*gvo_.rows()+ j) = vo_grid_(i,j);
        *(src1_v + i*gvo_.rows()+ j) = 1.0 - vo_grid_(i,j);
      }
    }    

    computeEDT(dst_v , src_v , gvo_.rows(), gvo_.cols());
    computeEDT(dst1_v, src1_v, gvo_.rows(), gvo_.cols());

    for (size_t i = 0; i < gvo_.rows(); i++)
    {
      for (size_t j = 0; j < gvo_.cols(); j++)
      {
        if(vo_grid_(i,j)==0)
          gvo_(i,j) =  (int)sqrt(*(dst_v  + i*gvo_.rows()+j));
        else
          gvo_(i,j) = -(int)sqrt(*(dst1_v + i*gvo_.rows()+j))+1;
      }
    }  

    free(src_v);    src_v  = NULL; 
    free(dst_v);    dst_v  = NULL;
    free(src1_v);   src1_v = NULL;
    free(dst1_v);   dst1_v = NULL;
  }

  bool Gvo::getState(const Eigen::Vector3d &v)
  {
    bool state = false;
    int count = 0;
    for (size_t i = 0; i < obs_num_; i++)
    {
      for (size_t j = 0; j < vo_[i].rows(); j++)
      {
        if(vo_[i](j,0)*v(0)+ vo_[i](j,1)*v(1) <= vo_[i](j,2))
        count = count + 1;
      }
      if(count == vo_[i].rows())
      {
        return state = true;
      }
      count = 0;    
    }
    return state;
  }

  double Gvo::calcVelDistance(const Eigen::Vector3d& v)
  {
    double dist;
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    getSurroundPts(v,sur_pts, diff);
    double dists[2][2];
    getSurroundDistance(sur_pts, dists);
    interpolateBilinearDist(dists, diff, dist);
    return dist;
  }

  Eigen::Vector2d Gvo::calcGrad(const Eigen::Vector3d& v)
  {
    Eigen::Vector2d dist_grad;
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    getSurroundPts(v,sur_pts, diff);
    double dists[2][2];
    getSurroundDistance(sur_pts, dists);
    interpolateBilinear(dists, diff, dist_grad);
    return dist_grad;
  }

  void Gvo::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector2d pts[2][2], 
                                Eigen::Vector2d& diff)
  {
    double dist_x = pos(0) - x_;
    double dist_y = y_ - pos(1);
    diff(0) = fmod(dist_x,map_resolution_);
    diff(1) = fmod(dist_y,map_resolution_);

    Eigen::Vector2d curr_index;
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

  void Gvo::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
  {
    for (size_t i = 0; i < 2; i++)
    {
      for (size_t j = 0; j < 2; j++)
      {
        Eigen::Vector2d tmp_index = pts[i][j];
        dists[i][j] = gvo_(tmp_index(0),tmp_index(1));   
      }
    }
  }

  void Gvo::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, 
                                Eigen::Vector2d& grad)
  {
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

  void Gvo::interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff,  
                    double& dist)
  {
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

  void Gvo::optimizeVel()
  {
    /* initialize solver */
    iter_num_        = 0;
    min_cost_        = std::numeric_limits<double>::max();
    int variable_num = 2;
    nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
    opt.set_min_objective(Gvo::costFunctionVel,this);
    opt.set_maxeval(200);
    opt.set_maxtime(0.02);
    opt.set_xtol_rel(1e-5);
    const double  bound = 2.0;
    vector<double> lb(variable_num), ub(variable_num);
    vector<double> q(variable_num);
    q[0] = v_local_(0); q[1] = v_local_(1);
    lb[0] = q[0]-bound; lb[1] = q[1]-bound;
    ub[0] = q[0]+bound;ub[1] = q[1]+bound;
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
    opt_vel_(0) = best_variable_[0];
    opt_vel_(1) = best_variable_[1];
    opt_vel_(2) = 0.0;

    // cout<< "optimize successfully~"<<endl;
    // cout<<"iter num :"<<iter_num_<<endl;
  }

  double Gvo::costFunctionVel(const std::vector<double>& x, std::vector<double>& grad,
                              void* func_data)
  {
    Gvo* opt = reinterpret_cast<Gvo*>(func_data);
    Eigen::MatrixXd grad_matrix;
    double cost;
    opt->combineCostVel(x,grad_matrix,cost);
    opt->iter_num_++;
    grad.resize(2);
    grad[0] =  grad_matrix(0,0);grad[1] =  grad_matrix(0,1);
    /* save the min cost result */
    if (cost < opt->min_cost_) {
      opt->min_cost_     = cost;
      opt->best_variable_ = x;
      //cout<<opt->best_variable_.size()<<endl;
    }
    return cost;
  } 

  void Gvo::combineCostVel(const std::vector<double>& x,Eigen::MatrixXd &grad, double &f_combine)
  {
      f_combine = 0.0;
      grad.resize(1,2);
      grad.setZero(1,2);//
      double f_pref, f_dist,f_direct;
      Eigen::MatrixXd g_pref_ = Eigen::MatrixXd::Zero(1, 2);
      Eigen::MatrixXd g_dist_ = Eigen::MatrixXd::Zero(1, 2);
      Eigen::MatrixXd g_direct = Eigen::MatrixXd::Zero(1, 2);
      f_pref = f_dist = f_direct = 0;
      Eigen::MatrixXd vel  = Eigen::MatrixXd::Zero(1, 2);
      vel(0,0) = x[0];  vel(0,1) = x[1];
      // cout << "vel is " <<vel<<endl;
      calcPrefCost(vel,f_pref,g_pref_);
      // calcGvoCost (vel,f_dist,g_dist_);
      calcGuideCost(vel,f_direct,g_direct);
      f_combine = K1_ * f_pref  + K2_ * f_dist  + K3_ * f_direct;
      grad      = K1_ * g_pref_ + K2_ * g_dist_ + K3_ * g_direct;
      // cout<<"-------------------------------------vel cost-------------------------------------"<<endl;
      // cout<< "f_pref is : "<< f_pref <<endl;
      // cout<< "f_dist is : "<< f_dist <<endl;
      // cout<< "f_direct is : "<< f_direct <<endl;
      // cout<<endl;
  } 

  void Gvo::calcPrefCost(const Eigen::MatrixXd &q, double &cost,Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    Eigen::Vector2d tmp_vel(q(0,0)-v_local_(0),q(0,1)-v_local_(1));
    cost = pow(tmp_vel.norm(),2);
    gradient = 2*tmp_vel.transpose();
  }
  
  void Gvo::calcGvoCost(const Eigen::MatrixXd &q, double &cost,Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    double  dist;
    Eigen::Vector2d dist_grad;
    Eigen::Vector3d tmp_v;
    if(fabs(q(0,0))>5.0 || fabs(q(0,1))>5.0)
    {
      ROS_WARN("UNREASONABLE VELOCITY!");
      return;
    }
    tmp_v(0) = q(0,0); tmp_v(1) = q(0,1); tmp_v(2) = 0.0;
    dist = calcVelDistance(tmp_v);
    dist_grad = calcGrad(tmp_v);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();
    if (dist < v_safe_) 
    {
      cost += pow(dist - v_safe_, 2);
      gradient += 2.0 * (dist - v_safe_) * dist_grad.transpose(); 
    }
  }

  void Gvo::calcGuideCost(const Eigen::MatrixXd &q, double &cost,Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    Eigen::Vector2d tmp_vel(q(0,0)-v_guide_(0),q(0,1)-v_guide_(1));
    cost = pow(tmp_vel.norm(),2);
    gradient = 2*tmp_vel.transpose();
  }

  void Gvo::calcGuideVel()
  {
    //TO MAKE VEL ESCAPE FROM LOCAL MINIMA
    v_guide_ = v_local_;
    //1. random points generation,use GVO & Direction to determine the candidate list
    //2. find the nearest vel as guide vel
    
    //1.
    int numPoints = 400;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-fabs(x_), fabs(x_));
    std::vector<Eigen::Vector2d> candidatepoints; 
    
    for (int i = 0; i < numPoints; ++i) 
    {
      Eigen::Vector2d point(dist(gen),dist(gen));
      candidatepoints.push_back(point);
    }

    // for (int i = 0; i < numPoints; ++i) {
    //     std::cout << "Points: (" << candidatepoints[i](0) << ", " << candidatepoints[i](1) << ")\n";
    // }

    //2.
    guidepoints.clear();
    near_guidepoints.clear();// IF CANT MOVE ON, THEN SEARCH SOME OTHER DIRECTION TO MAKE SURE AVOID COLLISION
    for (int i = 0; i < numPoints; ++i) 
    {
      Eigen::Vector2i index = posToIndex(candidatepoints[i]);

      if( gvo_(index(0),index(1)) > v_safe_ )
        {
          double cosine = candidatepoints[i].dot(target_point) / (candidatepoints[i].norm() * target_point.norm());
          if(cosine > sqrt(2)/2)//local direction to the front M_PI/4
            guidepoints.push_back(candidatepoints[i]);
          else
            near_guidepoints.push_back(candidatepoints[i]);
        }
    }

    if(guidepoints.size() == 0)
    {
      // cout<< "\033[1;32mNO GUIDE POINTS, USE near_guidepoints.\033[0m" << endl;

      double min_dis = 10000;
      Eigen::Vector2d v_local_2d(v_local_(0),v_local_(1));

      for (size_t i = 0; i < near_guidepoints.size(); i++)
      {
        double dis = (near_guidepoints[i]-v_local_2d).norm();
        if( dis< min_dis )
        {
          min_dis = dis;
          v_guide_ << near_guidepoints[i](0), near_guidepoints[i](1), 0.0;
        }
      }
    }

    else
    {
      // cout<< "\033[1;32mUSE guidepoints.\033[0m" << endl;

      double min_dis = 10000;
      Eigen::Vector2d v_local_2d(v_local_(0),v_local_(1));

      for (size_t i = 0; i < guidepoints.size(); i++)
      {
        double dis = (guidepoints[i]-v_local_2d).norm();
        if( dis< min_dis )
        {
          min_dis = dis;
          v_guide_ << guidepoints[i](0), guidepoints[i](1), 0.0;
        }
      }
    }

  }

}