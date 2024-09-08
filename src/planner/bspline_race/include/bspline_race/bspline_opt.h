#ifndef  _BSPLINE_OPT_H
#define  _BSPLINE_OPT_H

//Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

//STANDARD
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>
#include <string>
#include <memory>

//ROS
#include <ros/ros.h>

//Nlopt optimization
#include <nlopt.hpp>

//自定义
#include <bspline_race/UniformBspline.h>
// #include<plan_env/edt_environment.h>

using namespace std;

namespace FLAG_Race
{
       class bspline_optimizer
    {
        public:
            int cps_num_;//控制点个数
            int p_order_;//轨迹阶次
            int Dim_;//b样条维度
            double bspline_interval_;//节点向量时间间隔
            double beta_;
            UniformBspline u_;

            //从状态机读入的参数
            double lambda1_,lambda2_,lambda3_;// smooth cost, ESDF cost, feasibility cost
            double max_vel_;//最大速度
            double max_acc_;//最大加速度

            //从A star得到的路径点
            std::vector<Eigen::Vector2d> path_;
            
            //ESDF
            Eigen::MatrixXd esdf_map_;
            double map_resolution_;//地图分辨率
            double origin_x_, origin_y_;//地图起始点x,y，右手坐标系 ，最左下角的栅格子的中心
            double startPoint_x,startPoint_y;//找到地图左上角的点，x负，y正

            //nlopt optimizaiton
            Eigen::MatrixXd control_points_;
            int iter_num_;       // iteration of the solver
            int variable_num;//变量个数
            std::vector<double> best_variable_;  //nlopt最终输出
            double safe_distance_;//安全距离
            
            double min_cost_;       //
            int    algorithm1_ = 15;             // optimization algorithms for quadratic cost
            int    algorithm2_ = 11;             // optimization algorithms for general cost

            // std::shared_ptr<EDTEnvironment> edt_environment_;
            // EDTEnvironment::Ptr edt_environment_;
            
        public://函数
            bspline_optimizer() {}
            bspline_optimizer(const std::vector<Eigen::Vector2d> &path, const int&Dim,const int &p);
            bspline_optimizer(const int&Dim,const int &p,const double &dist);
            ~bspline_optimizer();

            //从状态机读入
            void setOptParam(const double lambda1,const double lambda2,const double lambda3,
                                                    const double safe_dist);
            void setMapParam(const double &origin_x,const double &origin_y, const double &map_resolution,
                                                    const double &start_x, const double &start_y);
            void setVelAcc(const double vel, const double acc);
            void setSmoothParam(const double lambda1,const double lambda2,
                                                            const double vel, const double acc);
            void setSplineParam(const UniformBspline &u);
            // void setEnvironment(const EDTEnvironment::Ptr& env);
            //从mapping读入
            void setEsdfMap(const Eigen::MatrixXd &esdf_map);
            void initialControlPoints(UniformBspline u);

            inline void setDimandOrder(const int&Dim, const int&p)
            {
                Dim_  = Dim;
                p_order_ = p;
            }
            inline void setPath(const std::vector<Eigen::Vector2d> &path)
            {
                path_.clear();
                path_ = path;
                cps_num_ = path.size() + 2*p_order_ -2;
            }

            //nlopt相关
            void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                                                    Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
            void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);
            void calcEsdfCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);   
            double calcDistance(const Eigen::MatrixXd &q);
            Eigen::Vector2d calcGrad(const Eigen::MatrixXd &q);   
            void getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff);
            void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
            void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad);
            void interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff, double& dist);    
            void combineCost( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine);
            void combineCostSmooth( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine);
            static double costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                                        void* func_data);
            static double costFunctionSmooth(const std::vector<double>& x, std::vector<double>& grad,
                                                        void* func_data);
            void optimize();//求解NLOPT，将最优变量保存为控制点  
            void optimize_withoutesdf();//对比
            void optimizesmooth();//const std::shared_ptr u
            template <typename T> std::vector<size_t> sort_indexes(std::vector<T> &v)//实现matlab mink函数
            {   
                std::vector<size_t> idx(v.size());
                iota(idx.begin(), idx.end(), 0);
                sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
                return idx;// 返回索引
            }
            template<typename T>  inline T lerp(const T &lo, const T &hi, float t)  { return (lo * (0.1 - t) + hi * t)*10; }
    };
}

#endif