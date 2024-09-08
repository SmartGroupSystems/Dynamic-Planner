#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; //坐标（真实，常为标准化后的
    Eigen::Vector3i dir;   // direction of expanding 扩展方向，为JPS准备
    Eigen::Vector3i index;//索引（格子图
	
    double gScore, fScore;//打分
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
      //USAGE  初始化函数
		id = 0;//均为open list
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();//不设置扩展方向

		gScore = inf;//评分设为无穷
		fScore = inf;
		cameFrom = NULL;//无先导结点
    }

    GridNode(){};
    ~GridNode(){};
};


#endif
