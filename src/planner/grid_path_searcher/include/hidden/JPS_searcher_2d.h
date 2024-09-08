#ifndef _JPS_SEARCHER_H_
#define _JPS_SEARCHER_H_

#include "Astar_searcher_2d.h"
#include "JPS_utils_2d.h"

class JPSPathFinder2d: public AstarPathFinder
{	
	private:
		bool isOccupied(const int & idx_x, const int & idx_y) const;
		bool isOccupied(const Eigen::Vector2i & index) const;
		bool isFree(const int & idx_x, const int & idx_y) const;
		bool isFree(const Eigen::Vector2i & index) const;
	public:
		JPS2DNeib * jn2d;

    	JPSPathFinder2d(){
    		jn2d = new JPS2DNeib();
    	};
    	
    	~JPSPathFinder2d(){
    		delete jn2d;
    	};
		void JPSGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
        bool hasForced(const Eigen::Vector2i & idx, const Eigen::Vector2i & dir);
        bool jump(const Eigen::Vector2i & curIdx, const Eigen::Vector2i & expDir, Eigen::Vector2i & neiIdx);
    	void JPSGraphSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);

		void JPSGraphSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt){
			ros::Time time_1 = ros::Time::now();    

   			//index of start_point and end_point
    		Vector2i start_idx = coord2gridIndex(start_pt);
    		Vector2i end_idx   = coord2gridIndex(end_pt);
    		goalIdx = end_idx;
	    	//position of start_point and end_point
    		start_pt = gridIndex2coord(start_idx);
    		end_pt   = gridIndex2coord(end_idx);
		}
};

#endif