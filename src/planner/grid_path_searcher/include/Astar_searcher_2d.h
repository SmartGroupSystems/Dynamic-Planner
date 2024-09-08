

#ifndef _ASTAR_SEARCHER_2D_H
#define _ASTAR_SEARCHER_2D_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
//#include "backward.hpp"
#include "node_2d.h"

class AstarPathFinder2d
{	
	private:

	protected:
		uint8_t * data;
		GridNodePtr2d ** GridNodeMap;
		std::vector<GridNodePtr2d> usedGridNode;
		Eigen::Vector2i goalIdx;
		int GLX_SIZE, GLY_SIZE, GLXY_SIZE;//, GLZ_SIZE;
		// int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution, interval_;
		double gl_xl, gl_yl;//, gl_zl;
		double gl_xu, gl_yu;//, gl_zu;
		// bool first_node_expanded;

		GridNodePtr2d terminatePtr;
		std::multimap<double, GridNodePtr2d> openSet;
		// std::multimap<double, GridNodePtr2d> closedSet;
		/* USAGE openSet
			multimap 类型，存储多键值对
			此处存储的键值对为：
					<double, GridNodePtr2d>{ Score  ,  Ptr }
			参考：http://c.biancheng.net/view/7190.html
		 */



		double getHeu(GridNodePtr2d node1, GridNodePtr2d node2);
		bool AstarGetSucc(GridNodePtr2d currentPtr, std::vector<GridNodePtr2d> & neighborPtrSets, std::vector<double> & edgeCostSets);		

    	// bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		// bool isOccupied(const Eigen::Vector3i & index) const;
		// bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		// bool isFree(const Eigen::Vector3i & index) const;

		bool isOccupied(const int & idx_x, const int & idx_y) const;
		bool isOccupied(const Eigen::Vector2i & index) const;
		bool isFree(const int & idx_x, const int & idx_y) const;
		bool isFree(const Eigen::Vector2i & index) const;
		
		// Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		// Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);



	public:
		// AstarPathFinder(){};
		// ~AstarPathFinder(){};
		// void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		// void resetGrid(GridNodePtr2d ptr);
		// void resetUsedGrids();

		// void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		// void setObs(const double coord_x, const double coord_y, const double coord_z);

		// Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		// std::vector<Eigen::Vector3d> getPath();
		// std::vector<Eigen::Vector3d> getVisitedNodes();
		
		AstarPathFinder2d(){};
		~AstarPathFinder2d(){};
		Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i & index);
		Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d & pt);
		bool getData(const Eigen::Vector2d & pt);
		bool getData(const Eigen::Vector2i & pt);
		bool arrived(const Eigen::Vector2d & pt1,const Eigen::Vector2d & pt2);
		bool AstarGraphSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);
		void resetGrid(GridNodePtr2d ptr);
		void resetUsedGrids();
		int twistTest(int i,int j);
		void cleanStartObs(Eigen::Vector2d _start_pt);

		void initGridMap(double _resolution, Eigen::Vector2d global_xyz_l, Eigen::Vector2d global_xyz_u, 
						 int max_x_id, int max_y_id, double interval);
		void setObs(const double coord_x, const double coord_y);
		void cleanObs();

		Eigen::Vector2d coordRounding(const Eigen::Vector2d & coord);
		std::vector<Eigen::Vector2d> getPath();
		// std::vector<Eigen::Vector2d> getTwist();
		std::vector<Eigen::Vector2d> getTwist2();
		std::vector<Eigen::Vector2d> getTwist3();
		std::vector<Eigen::Vector2d> getTwist4();
		std::vector<Eigen::Vector2d> getTwist_checkcolision(Eigen::Vector2d start_pt);
		std::vector<Eigen::Vector2d> getTwist_checkcolision2();
		std::vector<Eigen::Vector2d> getTwist_checkcolision3(Eigen::Vector2d start_pt);
		std::pair<bool,Eigen::Vector2i> check_collision(GridNodePtr2d start_ptr,GridNodePtr2d aim_ptr);
		std::pair<bool,Eigen::Vector2i> check_collision(Eigen::Vector2d start_ptr,Eigen::Vector2d aim_ptr);
		std::vector<Eigen::Vector2d> getVisitedNodes();


		Eigen::Vector2d start_pt_real;
};

#endif