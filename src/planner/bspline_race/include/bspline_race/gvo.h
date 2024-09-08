#ifndef  _GVO_H
#define  _GVO_H

//Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

//ROS
#include <ros/ros.h>

//Nlopt optimization
#include <nlopt.hpp>

//STANDARD
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>
#include <string>
#include <memory>
#include <random>

//自定义
#include <map_generator/dynamic_obs.h>
#include <bspline_race/EdtTransform.h>

using namespace std;

namespace FLAG_Race
{
    class Gvo
    {
        private:
            int map_x_,map_y_;// vel_grid_ size
            double x_,y_;// vel_grid_ start point, start from the upper left corner
            double map_resolution_;
            double delta_t_;
            double quad_r_;
            double inflate_r_;

        public:
            int obs_num_;
            vector<Eigen::Vector3d> obs_pos_;
            vector<Eigen::Vector3d> obs_vel_;
            vector<double> obs_size_;
            vector<Eigen::MatrixXd> vo_;
            Eigen::MatrixXd vo_grid_;
            Eigen::MatrixXd gvo_;
            Eigen::Vector3d opt_vel_;
            Eigen::Vector3d v_local_;
            Eigen::Vector3d v_guide_;
            std::vector<Eigen::Vector2d> guidepoints;
            std::vector<Eigen::Vector2d> near_guidepoints;
            Eigen::Vector2d target_point; 

        public:
            int iter_num_;       // iteration of the solver
            std::vector<double> best_variable_;  //
            double min_cost_;
            double K1_,K2_,K3_;//
            double v_safe_;  
            bool use_guide_points = true;

        public:  
            Gvo(){}
            Gvo(const int &rows, const int &cols, const double &x,
                const double &y, const double &delta_t,const double &r);
                //Gvo(map size x , map size y; index x, index y, start from the upper left corner
                //delta t, quad radius r)
            ~Gvo(){}
            void getObsInfo(const map_generator::dynamic_obs &info);
            void getLocalVel(const Eigen::Vector3d &v);
            void getParam(const double K1,const double K2,const double K3,const double v_safe);
            bool getState(const Eigen::Vector3d &v);
            void setVo();
            void setVoGrid();
            void setGvo();
            inline void getTargetpoint(Eigen::Vector3d point)
            {
               target_point << point(0), point(1);                 
            }
            double calcVelDistance(const Eigen::Vector3d& v);
            Eigen::Vector2d calcGrad(const Eigen::Vector3d& v);
            void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector2d pts[2][2], 
                                Eigen::Vector2d& diff);
            void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
            void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, 
                                Eigen::Vector2d& grad);
            void interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff,  
                                double& dist);
            void optimizeVel();
            static double costFunctionVel(const std::vector<double>& x, std::vector<double>& grad,
                                                        void* func_data); 
            void combineCostVel( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine); 
            void calcGuideVel();
            void calcPrefCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);
            void calcGvoCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);    
            void calcGuideCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);         
            template<typename T>  inline T lerp(const T &lo, const T &hi, float t)  
            { return (lo * (0.1 - t) + hi * t)*10; }
            template <typename T> std::vector<size_t> sort_indexes(std::vector<T> &v)//matlab mink
            {   
                std::vector<size_t> idx(v.size());
                iota(idx.begin(), idx.end(), 0);
                sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
                return idx;// return index
            }

            inline Eigen::Vector2i posToIndex(const Eigen::Vector2d &pos)
            {
                Eigen::Vector2i curr_index;
                double dist_x = pos(0) - x_;
                double dist_y = y_ - pos(1);
                curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
                return curr_index;
            }


    };
}

#endif