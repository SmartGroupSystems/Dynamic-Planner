#include <bspline_race/UniformBspline.h>

namespace FLAG_Race

{
    UniformBspline::UniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        initUniformBspline(p, n,beta, D, s_ini, s_ter); 
    }

    UniformBspline::~UniformBspline() {}

    void UniformBspline::initUniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        p_ = p; 
        n_ = n-1;
        beta_ = beta;
        D_ =D;
        m_ = p_+n_+1;
        u_ = Eigen::VectorXd::Zero(m_ + 1); //u0 ~ um 共m+1个
        control_points_ = Eigen::MatrixXd::Zero(n_+1,D_);
        for(int i = 0; i<=m_; i++)
        {
            u_(i) = i;
        }
        s_ini_ = s_ini;
        s_ter_ = s_ter;
        setIniTerMatrix();
        getAvailableSrange();
        getAvailableTrange();
        getInterval();
    }

    void UniformBspline::setIniTerMatrix()
    {
        A_ini.resize(3,3);
        A_ter.resize(3,3);
        A_ini << 1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
        A_ter<<1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
    }

    void UniformBspline::setControlPoints(const Eigen::MatrixXd &ctrl_points)
    {
        control_points_ = ctrl_points;
    }
    
    Eigen::MatrixXd UniformBspline::getTrajectory(const Eigen::VectorXd &t)
    {
        double u_probe;
        int t_size = t.size();
        Eigen::MatrixXd trajectory(t_size,D_);
        for (size_t i = 0; i < t_size; i++)
        {
            //map t(i) to uniform knot vector
            u_probe = t(i) * beta_ + u_(p_);
            trajectory.row(i) = singleDeboor(u_probe); 
        }
        return trajectory;
    }

     Eigen::Vector2d UniformBspline::singleDeboor(const double &u_probe)//the deboor's algorithm
     {  
        //bound the u_probe
        double u_probe_;
        int k;
        u_probe_ = min(max( u_(p_) , u_probe), u_(m_-p_));
        k = p_;
        while(true)
        {
            if(u_(k+1)>=u_probe_)
                break;
            k = k+1;
        }
        // t(t_ctt) is maped to knot segment u_k ~ u_{k+1}
        // there are at most p+1 basis functions N_k-p,p(u), N_k-p+1,p(u),..., N_k,p(u) non-zero on knot span [uk,uk+1)
        //the effective control points are P_k-p ~ P_k
        // since MATLAB index start from 1 instead of 0
        // The effective control points are
        double alpha;
        Eigen::MatrixXd d(p_+1,2);
        d = control_points_.block(k-p_,0,p_+1,2);// c++这里是从0行0列开始
        for (size_t i = 0; i < p_; i++)
        {
            for (size_t j = p_; j > i; j--)
            {
                alpha = (u_probe_ - u_(j+k-p_)) /(u_(j+k-i) - u_(j+k-p_)); 
                d.row(j) = (1 - alpha)*d.row(j-1) + alpha*d.row(j);
            }          
        }

            Eigen::Vector2d value;
            value = d.row(p_);
            return value;
     }

    void UniformBspline::getAvailableSrange()
    {
        s_range = {u_(p_),u_(m_-p_)};
    }

    void UniformBspline::getAvailableTrange()
    {
        t_range = {0/beta_, (u_(m_-p_)-u_(p_))/beta_};
    }

    void UniformBspline::getInterval()
    {
        interval_ = (u_(1) - u_(0))/beta_;
    }

    void UniformBspline::getT(const int &trajSampleRate)
    {
        time_.resize((t_range(1)-t_range(0))*trajSampleRate+1);
        
        for (size_t i = 0; i < time_.size(); i++)
        {
            time_(i) = t_range(0) + i*(1.0/trajSampleRate);
        }
    }

    UniformBspline UniformBspline::getDerivative()
    {     
            UniformBspline spline(p_,n_,beta_,D_,s_ini_,s_ter_);
            spline.p_ = spline.p_ -1;
            spline.m_ = spline.p_ +spline.n_ +1;
            spline.u_.resize(u_.size()-2);
            spline.u_ = u_.segment(1,m_-1);//从第2个元素开始的m-1个元素
            spline.control_points_.resize(control_points_.rows()-1,D_);
            for (size_t i = 0; i < spline.control_points_.rows(); i++)
            {
                spline.control_points_.row(i) = spline.beta_*(control_points_.row(i+1) - control_points_.row(i));
            } 
            spline.time_ = time_;
            return spline;
    }

    Eigen::VectorXd UniformBspline::getBoundConstraintb()
    {
        int nm = (n_+1)*D_;
        Eigen::VectorXd b= Eigen::VectorXd::Zero(nm);
        Eigen::MatrixXd tmp1(3,2);//前三个控制点的值
        Eigen::MatrixXd tmp2(3,2);//末尾三个控制点的值
        // solve Ax = b
        tmp1 = A_ini.colPivHouseholderQr().solve(s_ini_);
        tmp2 = A_ter.colPivHouseholderQr().solve(s_ter_);
         for (size_t j = 0; j< D_; j++)
        {
            for (size_t i = 0; i < 3; i++)
            {
                b(i+j*(n_+1)) = tmp1(i,j);
                b((j+1)*(n_+1)-i-1) = tmp2(3-i-1,j);
            }      
        }    
        return b;   
    }

}
