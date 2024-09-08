#ifndef _NODE_2D_H_
#define _NODE_2D_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <vector>
//#include "backward.hpp"

#define inf 1>>20
struct GridNode2d;
typedef GridNode2d* GridNodePtr2d;

struct GridNode2d
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector2d coord; //坐标（真实，常为标准化后的
    Eigen::Vector2i dir;   // direction of expanding 扩展方向，为JPS准备?
    Eigen::Vector2i index;//索引（格子图
	
    double gScore, fScore;//打分
    GridNodePtr2d cameFrom;
    bool obs_around = false;
    // GridNodePtr2d twistFrom;
    // int cameFrom_Slash = 0;
    std::multimap<double, GridNodePtr2d>::iterator nodeMapIt;

    GridNode2d(Eigen::Vector2i _index, Eigen::Vector2d _coord){  
      //USAGE  初始化函数
		id = 0;//均为open list
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector2i::Zero();//不设置扩展方向

		gScore = inf;//评分设为无穷
		fScore = inf;
		cameFrom = NULL;//无先导结点
    obs_around = false;
    }

    GridNode2d(){};
    ~GridNode2d(){};
};


#endif
