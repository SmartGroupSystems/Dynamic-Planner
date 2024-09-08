#include "Astar_searcher_2d.h"

using namespace std;
using namespace Eigen;
bool tie_break = false;
// GridNodePtr2d firstPtr  = NULL;
void AstarPathFinder2d::initGridMap(double _resolution, Vector2d global_xyz_l, Vector2d global_xyz_u, 
                                    int max_x_id, int max_y_id, double interval)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    // gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    // gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    // GLZ_SIZE = max_z_id;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;
    // GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    // GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    interval_ = interval;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXY_SIZE];
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr2d * [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr2d  [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            Vector2i tmpIdx(i,j);
            Vector2d pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode2d(tmpIdx, pos);
        }
    }
}

void AstarPathFinder2d::resetGrid(GridNodePtr2d ptr)
{
    //USAGE 将点ptr的id置0，即不在open/closedlist中
    ptr->id = 0;
    ptr->cameFrom = NULL;
    // ptr->twistFrom = NULL;
    // first_node_expanded = false;
    // firstPtr = NULL;
    // ptr->cameFrom_Slash = 0;
    ptr->gScore = inf;
    ptr->fScore = inf;
    ptr->obs_around = false;
}

void AstarPathFinder2d::resetUsedGrids()
{   
    for(int i = 0; i < usedGridNode.size(); i++)
        resetGrid(usedGridNode[i]);
    usedGridNode.clear();
}

void AstarPathFinder2d::setObs(const double coord_x, const double coord_y)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl || 
        coord_x >= gl_xu || coord_y >= gl_yu )
        {
            ROS_WARN("out range");
            return;//检测超越边界
        }
        

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);

    //中心点
    data[idx_x * GLY_SIZE + idx_y] = 1;

    //膨胀一圈
    data[(idx_x-1)  * GLY_SIZE + idx_y+1]    = 1;
    data[(idx_x-1)  * GLY_SIZE + idx_y]      = 1;
    data[(idx_x-1)  * GLY_SIZE + idx_y-1]    = 1;
    data[idx_x         * GLY_SIZE + idx_y+1] = 1;
// data[idx_x         * GLY_SIZE + idx_y] = 1;
    data[idx_x         * GLY_SIZE + idx_y-1] = 1;
    data[(idx_x+1) * GLY_SIZE + idx_y+1]     = 1;
    data[(idx_x+1) * GLY_SIZE + idx_y]       = 1;
    data[(idx_x+1) * GLY_SIZE + idx_y-1]     = 1;
    

}
void AstarPathFinder2d::cleanObs()
{   
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
}
void AstarPathFinder2d::cleanStartObs(Eigen::Vector2d _start_pt){
    Eigen::Vector2i pt;
    pt = coord2gridIndex(_start_pt);
    data[pt(0) * GLY_SIZE + pt(1)] = 0;
}

vector<Vector2d> AstarPathFinder2d::getVisitedNodes()
{   
    vector<Vector2d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
            if(GridNodeMap[i][j]->id == -1)  // visualize nodes in close list only
            visited_nodes.push_back(GridNodeMap[i][j]->coord);
        }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector2d AstarPathFinder2d::gridIndex2coord(const Vector2i & index) 
{
    Vector2d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;

    return pt;
}

Vector2i AstarPathFinder2d::coord2gridIndex(const Vector2d & pt) 
{
    //USAGE 点云坐标数据转换为栅格地图坐标
    //
    Vector2i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
                  min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector2d AstarPathFinder2d::coordRounding(const Eigen::Vector2d & coord)
{
    //USAGE 从真实坐标到栅格地图再到真实坐标
    //从而使得可视化的地图为正正经经的方格
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder2d::isOccupied(const Eigen::Vector2i & index) const
{
    return isOccupied(index(0), index(1));
}

inline bool AstarPathFinder2d::isFree(const Eigen::Vector2i & index) const
{
    return isFree(index(0), index(1));
}

inline bool AstarPathFinder2d::isOccupied(const int & idx_x, const int & idx_y) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && 
            (data[idx_x * GLY_SIZE + idx_y] == 1));
}

inline bool AstarPathFinder2d::isFree(const int & idx_x, const int & idx_y) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE  && 
           (data[idx_x * GLY_SIZE + idx_y] < 1));
}
bool AstarPathFinder2d::getData(const Eigen::Vector2d & pt)
{
    return isOccupied(coord2gridIndex(pt));
}
bool AstarPathFinder2d::getData(const Eigen::Vector2i & pt)
{
    return isOccupied(pt);
}
bool AstarPathFinder2d::arrived(const Eigen::Vector2d & pt1,const Eigen::Vector2d & pt2)
{
    Vector2i pt_1 = coord2gridIndex(pt1);
    Vector2i pt_2 = coord2gridIndex(pt2);
    if(pt_1 == pt_2) return true;
    else return false;
}

//TODO 
//AstarGetSucc函数
// 获取该点周围的所有节点和周围点的edgeCostSets(edgeCostSets:该点到目标的的距离)
inline bool AstarPathFinder2d::AstarGetSucc(GridNodePtr2d currentPtr, vector<GridNodePtr2d> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear(); // Note: the pointers in this set copy pointers to GridNodeMap
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder2d::AstarGetSucc yourself 
    please write your code below
    *
    */
    // idea index -> coordinate -> edgecost
    if(currentPtr == nullptr)
        std::cout << "Error: Current pointer is null!" << endl;


    Eigen::Vector2i thisNode = currentPtr -> index;
    int this_x = thisNode[0];
    int this_y = thisNode[1];
    auto this_coord = currentPtr -> coord;
    int  n_x, n_y;
    double dist;
    GridNodePtr2d temp_ptr = nullptr;
    Eigen::Vector2d n_coord;
    bool obs_around = false;
    // 遍历周围点,获取周围点的edgeCostSets
    for(int i = -1;i <= 1;++i ){
        for(int j = -1;j <= 1;++j ){
                if( i == 0 && j == 0)
                    continue; // to avoid this node

                n_x = this_x + i;
                n_y = this_y + j;

                if( (n_x < 0) || (n_x > (GLX_SIZE - 1)) || (n_y < 0) || (n_y > (GLY_SIZE - 1) ) )
                    continue; // to avoid index problem

                if(isOccupied(n_x, n_y))
                {
                    obs_around = true;
                    continue; // to avoid obstacles
                }
                    

                // put the pointer into neighborPtrSets
                temp_ptr = GridNodeMap[n_x][n_y];

                if(temp_ptr->id == -1) 
                    continue; // TODO to check this; why the node can transversing the obstacles
                if((i*i+j*j)==2){
                    if(isOccupied(n_x-i, n_y)&&isOccupied(n_x, n_y-j)) continue;
                }
                //Twist
                // temp_ptr->cameFrom_Slash = twistTest(i,j);
                // if(temp_ptr->id == 0) temp_ptr->cameFrom_Slash = -1;

                n_coord = temp_ptr->coord;

                if(temp_ptr == currentPtr){
                    std::cout << "Error: temp_ptr == currentPtr)" << std::endl;
                }

                if( (std::abs(n_coord[0] - this_coord[0]) < 1e-6) and (std::abs(n_coord[1] - this_coord[1]) < 1e-6) ){
                    std::cout << "Error: Not expanding correctly!" << std::endl;
                    std::cout << "n_coord:" << n_coord[0] << " "<<n_coord[1]<< std::endl;
                    std::cout << "this_coord:" << this_coord[0] << " "<<this_coord[1] << std::endl;

                    std::cout << "current node index:" << this_x << " "<< this_y<< std::endl;
                    std::cout << "neighbor node index:" << n_x << " "<< n_y<< std::endl;
                }

                dist = std::sqrt( (n_coord[0] - this_coord[0]) * (n_coord[0] - this_coord[0])+
                        (n_coord[1] - this_coord[1]) * (n_coord[1] - this_coord[1]));
                
                neighborPtrSets.push_back(temp_ptr); // calculate the cost in edgeCostSets: inf means that is not unexpanded
                edgeCostSets.push_back(dist); // put the cost inot edgeCostSets


        }
    }
    return obs_around;
}

// 启发函数 getHeu
double AstarPathFinder2d::getHeu(GridNodePtr2d node1, GridNodePtr2d node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder2d::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    double h;
    auto node1_coord = node1->coord;
    auto node2_coord = node2->coord;

    // Heuristics 1: Manhattan
    // h = std::abs(node1_coord(0) - node2_coord(0) ) +
    //     std::abs(node1_coord(1) - node2_coord(1) ) +
    //     std::abs(node1_coord(2) - node2_coord(2) );

    // Heuristics 2: Euclidean
    // h = std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) +
    //     std::pow((node1_coord(1) - node2_coord(1)), 2 ));

    // Heuristics 3: Diagnol distance
    double dx = std::abs(node1_coord(0) - node2_coord(0) );
    double dy = std::abs(node1_coord(1) - node2_coord(1) );
    // dx+dy+xz = Manhattan
    double min_xy = std::min({dx, dy});
    h = dx + dy  + (std::sqrt(2.0) -2) * min_xy; // idea: diagnol is a short-cut, find out how many short-cuts can be realized

    //这个tie_break目前还没搞懂什么意思
    if(tie_break){
        double p = 1.0 / 25.0;
        h *= (1.0 + p);
        //std::cout << "Tie Break!" << std::endl;
    }

    return h;
}


//TODO 寻找路径函数
// A*路径搜索
bool AstarPathFinder2d::AstarGraphSearch(Vector2d start_pt, Vector2d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector2i start_idx = coord2gridIndex(start_pt);
    Vector2i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr2d startPtr = new GridNode2d(start_idx, start_pt);
    GridNodePtr2d endPtr   = new GridNode2d(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    //currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr2d currentPtr  = NULL;
    GridNodePtr2d neighborPtr = NULL;
    

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder2d::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    usedGridNode.push_back(startPtr);
    // make_pair的用法:无需写出型别,就可以生成一个pair对象;比如std::make_pair(42, '@'),而不必费力写成：std::pair<int, char>(42, '@')
    //todo Note: modified, insert the pointer GridNodeMap[i][j][k] to the start node in grid map
    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); 
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    // three dimension pointer GridNodeMap[i][j][k] is pointed to a struct GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord);
    // assign g(xs) = 0, g(n) = inf (already done in initialzation of struct)
    // mark start point as visited(expanded) (id 0: no operation, id: 1 in OPEN, id -1: in CLOSE )

    // 这个到底需不需要???测试后发现不需要也可以实现功能
    // JS: 标识该结点已进入openset
    GridNodeMap[start_idx[0]][start_idx[1]] -> id = 1;
    usedGridNode.push_back(GridNodeMap[start_idx[0]][start_idx[1]]);

    vector<GridNodePtr2d> neighborPtrSets;
    vector<double> edgeCostSets;
    Eigen::Vector2i current_idx; // record the current index

    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        //JS change
        
        // for (auto iter = openSet.begin(); iter != openSet.end(); ++iter) {
        //     cout << iter->first << " " << iter->second << endl;
        // }
        //JS change end


        // openset:待访问节点容器;closed set:访问过节点容器
        // openSet.begin()返回👈已经排好顺序的第一个键值对的双向迭代器
        // 此处xyz = openset中第一个结点的第二个键值（Ptr）的格子图坐标
        int x = openSet.begin()->second->index(0); 
        int y = openSet.begin()->second->index(1); 
        openSet.erase(openSet.begin());//删除指定键值对
        currentPtr = GridNodeMap[x][y];//当前👈
        // if(!first_node_expanded){
        //     //使得第一个节点👈自己
        //     currentPtr->twistFrom = currentPtr;
        //     first_node_expanded = true;
        //     firstPtr = currentPtr;
        // }
        

        // 如果节点被访问过;则返回
        if(currentPtr->id == -1)
            continue;
        // 标记id为-1,表示节点已被扩展
        currentPtr->id = -1;
        usedGridNode.push_back(currentPtr);

        // currentPtr = openSet.begin() -> second; // first T1, second T2
        // openSet.erase(openSet.begin()); // remove the node with minimal f value
        // current_idx = currentPtr->index;
        // GridNodeMap[current_idx[0]][current_idx[1]][current_idx[2]] -> id = -1;// update the id in grid node map
        
        // if the current node is the goal 
        if( currentPtr->index == goalIdx )
        {
            // 到达目标点，退出循环
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            // terminatePtr->twistFrom = currentPtr->cameFrom;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return 1;
        }
            
        //get the succetion
        bool obs_around = AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder2d::AstarGetSucc yourself     
        currentPtr->obs_around = obs_around;
        // 获取该点周围的所有节点和周围点的edgeCostSets(edgeCostSets:该点到目标的的距离)


        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            //遍历邻居节点
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                // shall update: gScore = inf; fScore = inf; cameFrom = NULL, id, mayby direction
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];//步数损失
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);//启发式
                neighborPtr->cameFrom = currentPtr; // todo shallow copy or deep copy
                // if(currentPtr->cameFrom_Slash!=neighborPtr->cameFrom_Slash){
                //     //若发生转折
                //     neighborPtr->twistFrom = currentPtr;
                // }
                // else{
                //     neighborPtr->twistFrom = currentPtr->twistFrom;
                // }

                // push node "m" into OPEN
                openSet.insert(make_pair(neighborPtr -> fScore, neighborPtr));
                neighborPtr -> id = 1;
                usedGridNode.push_back(neighborPtr);
                continue;
            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                // shall update: gScore; fScore; cameFrom, mayby direction
                if( neighborPtr->gScore > (currentPtr->gScore+edgeCostSets[i]))
                {//从当前节点currentPtr走更近一些
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                    neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr,endPtr);
                    neighborPtr -> cameFrom = currentPtr;
                //     if(currentPtr->cameFrom_Slash!=neighborPtr->cameFrom_Slash){
                //     //若发生转折
                //     neighborPtr->twistFrom = currentPtr;
                // }
                // else{
                //     neighborPtr->twistFrom = currentPtr->twistFrom;
                // }
                
                }

                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                // todo nothing to do here?
                continue;
            }
        }      
    }
    ros::Time time_2 = ros::Time::now();
    ROS_ERROR("Time consume in Astar path finding is %f.", (time_2 - time_1).toSec() );
    //if search fails
    ROS_ERROR("[Astar] No Grid need to be searched. Pathfinding process ended prematurely.");
    return 0;
}

// vector<Vector2d> AstarPathFinder2d::getTwist(){
//     vector<Vector2d> path;
//     vector<GridNodePtr2d> gridPath;

//     auto ptr = terminatePtr;
//     while(ptr -> twistFrom != firstPtr){
//         gridPath.push_back(ptr);
//         ptr = ptr->twistFrom;
//         //         thisNode = ptr -> index;
//         // twistTest()
//     }
//     gridPath.push_back(firstPtr);
//     for (auto ptr: gridPath)
//         path.push_back(ptr->coord);
        
//     reverse(path.begin(),path.end());

//     return path;
// }

vector<Vector2d> AstarPathFinder2d::getTwist2(){
    vector<Vector2d> path;
    vector<GridNodePtr2d> gridPath;
    Eigen::Vector2i thisNode;
    Eigen::Vector2i lastNode;
    Eigen::Vector2i ori;
    int this_twist;
    int last_twist;
    bool twist_confirm;
    // gridPath.push_back(terminatePtr);    
    auto ptr = terminatePtr;
    auto ptr_ = terminatePtr;
    //放入终止点
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
    while(ptr -> cameFrom -> cameFrom != NULL){
        // gridPath.push_back(ptr);
        thisNode = ptr -> index;
        ptr_ = ptr->cameFrom;
        lastNode = ptr_ -> index;
        ori = thisNode - lastNode;
        this_twist = twistTest(ori[0],ori[1]);
        // if(ptr!=NULL) lastNode = ptr -> index;
        // else break;
        twist_confirm = (last_twist != this_twist);
        if(twist_confirm) gridPath.push_back(ptr);
        last_twist = this_twist;
        ptr = ptr_;
    }
    // 放入起始点
    gridPath.push_back(ptr);
    // gridPath.push_back(ptr);
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}

vector<Vector2d> AstarPathFinder2d::getTwist3(){
    vector<Vector2d> path;
    vector<GridNodePtr2d> gridPath;
    Eigen::Vector2i thisNode;
    Eigen::Vector2i lastNode;
    Eigen::Vector2i ori;
    int this_twist;
    int last_twist;
    bool twist_confirm;
    // gridPath.push_back(terminatePtr);    
    auto ptr = terminatePtr;
    auto ptr_ = terminatePtr;
    //放入终止点
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
    int count_length = 1;
    if(ptr->cameFrom!=NULL){
        while(ptr -> cameFrom -> cameFrom != NULL){
            // gridPath.push_back(ptr);
            thisNode = ptr -> index;
            ptr_ = ptr->cameFrom;
            lastNode = ptr_ -> index;
            ori = thisNode - lastNode;
            this_twist = twistTest(ori[0],ori[1]);
            // if(ptr!=NULL) lastNode = ptr -> index;
            // else break;
            twist_confirm = (last_twist != this_twist);
            if(twist_confirm || count_length == 3) 
            {
                gridPath.push_back(ptr);
                count_length = 0;
            }
            count_length++;
            last_twist = this_twist;
            ptr = ptr_;
        }
    }
    // 放入起始点
    cout<<gridPath.size()<<endl;
    gridPath.push_back(ptr);
    // gridPath.push_back(ptr);
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(),path.end());
    

    return path;
}

vector<Vector2d> AstarPathFinder2d::getTwist4(){
    vector<Vector2d> path;
    vector<GridNodePtr2d> gridPath;
    Eigen::Vector2i thisNode;
    Eigen::Vector2i lastNode;
    Eigen::Vector2i ori;
    int this_twist;
    int last_twist;
    bool twist_confirm; 
    auto ptr = terminatePtr;
    auto ptr_ = terminatePtr;
    //放入终止点
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
    int count_length = 1;
    if(ptr->cameFrom!=NULL){
        while(ptr -> cameFrom -> cameFrom != NULL){
            // gridPath.push_back(ptr);
            thisNode   = ptr -> index;
            ptr_       = ptr->cameFrom;
            lastNode   = ptr_ -> index;
            ori        = thisNode - lastNode;
            this_twist = twistTest(ori[0],ori[1]);
            twist_confirm = (last_twist != this_twist);
            if(twist_confirm) 
            {
                gridPath.push_back(ptr);
                count_length = 0;
            }
            count_length++;
            last_twist = this_twist;
            ptr        = ptr_;
        }
    }
    // 放入起始点
    ptr->coord = start_pt_real;
    gridPath.push_back(ptr);
    reverse(gridPath.begin(),gridPath.end());
    // 重新计算

    // cout<<"[start]"<<endl;
    for(auto iter=gridPath.begin();iter!=gridPath.end()-1;iter++)
    {
        // cout<<"[into_iter]"<<endl;
        auto start_ptr = *iter;
        auto end_ptr   = *(iter + 1);
        Vector2d vector_s2e;
        Vector2d vector_online;
        double lengh_of_line;
        int count_insert;
            // cout<<"start_ptr====\n"<<start_ptr->coord<<"====\n"<<endl;

        vector_s2e = end_ptr->coord - start_ptr->coord;
        lengh_of_line = vector_s2e.norm();
        count_insert = (int)(lengh_of_line/(double)(0.999));
        // cout<<"~~~~"<<endl;
        // cout<<"~~"<<lengh_of_line<<"~~"<<endl;
        // cout<<"~~"<<count_insert<<"~~"<<endl;
        // cout<<"~~~~"<<endl;
        for(int i = 0;i<=count_insert;i++)
        {
            vector_online = start_ptr->coord + 1.0*i*vector_s2e/vector_s2e.norm();
            // cout<<"vector_online====\n"<<vector_online<<"====\n"<<endl;
            path.push_back(vector_online);
        }
    }
    auto end_ptr = *(gridPath.end()-1);
    path.push_back(end_ptr->coord);
    // reverse(path.begin(),path.end());
    return path;
}


vector<Vector2d> AstarPathFinder2d::getTwist_checkcolision(Eigen::Vector2d start_pt){
    vector<Vector2d> path;
    vector<GridNodePtr2d> gridPath_;
    vector<GridNodePtr2d> gridPath;
    auto ptr = terminatePtr;
    // 轨迹缩减
    while(ptr -> cameFrom != NULL){
        gridPath_.push_back(ptr);
        ptr = ptr->cameFrom;
    }
    reverse(gridPath_.begin(),gridPath_.end());
    
    cout <<"[debug] current target: "<< (*(gridPath_.end()-1))->coord<<endl;
    cout <<"[debug] last ptr: "<< ptr->coord<<endl;
    GridNode2d real_start_pt;
    real_start_pt = *(*(gridPath_.begin()));
    real_start_pt.coord = start_pt;
    gridPath.push_back(&real_start_pt);
    auto start_pt_ = *(gridPath_.begin());
    while(!gridPath_.empty())
    {
        int id = 0, longest_id = 0;
        Eigen::Vector2i longest_colipoint(start_pt_->index);
        for(auto iter = gridPath_.begin(); iter != gridPath_.end(); iter++)
        {
            auto gridnode_ptr = *(iter);
            std::pair<bool,Eigen::Vector2i> check_point = check_collision(gridnode_ptr,start_pt_);
            if(!check_point.first)
                longest_id = id;
            id++;
        }
        cout<<"\033[0m"<<endl;
        // rate_.sleep();
        cout << "[debug] current longest id: "<<longest_id<<endl;
        cout << "[debug] current length: "<<gridPath_.size()<<endl;
        auto iter = gridPath_.begin();
        iter = iter + longest_id;
        gridPath.push_back(*(iter));
        start_pt_ = *(iter);
        gridPath_.erase(gridPath_.begin(),iter+1);
    }
    cout <<"[debug] current set target: "<< (*(gridPath.end()-1))->coord<<endl;

    // 轨迹补全
    bool fk_flag = false;
    auto iter_head = gridPath.begin();
    int j = 0;
    for(auto iter = gridPath.begin();iter!=gridPath.end();iter++)
    {
        auto start_ptr = *(iter_head);
        auto end_ptr   = *(iter);
        if((*iter_head)->coord == (*iter)->coord) continue;
        else
        {
            double lengh_of_line;
            int count_insert;
            Vector2d vector_s2e = (end_ptr->coord - start_ptr->coord);
            lengh_of_line = vector_s2e.norm();
            count_insert = (int)(lengh_of_line/(double)(interval_));
            for(int i = 0;i<=count_insert;i++)
            {
                Vector2d vector_online = start_ptr->coord + interval_*i*vector_s2e/vector_s2e.norm();
                // cout << "line[ "<< j <<"], number [ "<< i << "]:\n"<<vector_online<<endl;
                
                path.push_back(vector_online);
            }
            j++;
            iter_head = iter;
        }
    }
    path.push_back((*(gridPath.end()-1))->coord);
    return path;
}

vector<Vector2d> AstarPathFinder2d::getTwist_checkcolision2(){
    vector<Vector2d> path;
    vector<GridNodePtr2d> gridPath_;
    vector<GridNodePtr2d> gridPath;
    auto ptr = terminatePtr;
    // 轨迹缩减
    cout <<"--------------getTwist_checkcolision---------------"<<endl;
    gridPath_.push_back(ptr);
    while(ptr -> cameFrom != NULL){
        if(ptr -> obs_around)
            gridPath_.push_back(ptr);
        ptr = ptr->cameFrom;
    }
    reverse(gridPath_.begin(),gridPath_.end());
    gridPath.push_back(*(gridPath_.begin()));
    auto start_pt_ = *(gridPath_.begin());
    while(!gridPath_.empty())
    {
        int id = 0, longest_id = 0;
        Eigen::Vector2i longest_colipoint(start_pt_->index);
        for(auto iter = gridPath_.begin(); iter != gridPath_.end(); iter++)
        {
            auto gridnode_ptr = *(iter);
            std::pair<bool,Eigen::Vector2i> check_point = check_collision(gridnode_ptr,start_pt_);
            if(!check_point.first)
                longest_id = id;
            id++;
        }
        cout<<"\033[0m"<<endl;
        // rate_.sleep();
        auto iter = gridPath_.begin();
        iter = iter + longest_id;
        gridPath.push_back(*(iter));
        cout << "[debug] current tunning coord: "<<(*(iter))->coord<<endl;
        start_pt_ = *(iter);
        gridPath_.erase(gridPath_.begin(),iter+1);
    }

    // 轨迹补全
    bool fk_flag = false;
    auto iter_head = gridPath.begin();
    int j = 0;
    for(auto iter = gridPath.begin();iter!=gridPath.end();iter++)
    {
        auto start_ptr = *(iter_head);
        auto end_ptr   = *(iter);
        if((*iter_head)->coord == (*iter)->coord) continue;
        else
        {
            double lengh_of_line;
            int count_insert;
            Vector2d vector_s2e = (end_ptr->coord - start_ptr->coord);
            lengh_of_line = vector_s2e.norm();
            count_insert = (int)(lengh_of_line/(double)(interval_));
            for(int i = 0;i<=count_insert;i++)
            {
                Vector2d vector_online = start_ptr->coord + interval_*i*vector_s2e/vector_s2e.norm();
                // cout << "line[ "<< j <<"], number [ "<< i << "]:\n"<<vector_online<<endl;
                
                path.push_back(vector_online);
            }
            j++;
            iter_head = iter;
        }
    }
    return path;
}

vector<Vector2d> AstarPathFinder2d::getTwist_checkcolision3(Eigen::Vector2d start_pt){
    vector<Vector2d> path;
    vector<GridNodePtr2d> gridPath_;
    vector<GridNodePtr2d> gridPath;
    auto ptr = terminatePtr;
    // 轨迹缩减
    while(ptr -> cameFrom != NULL){
        gridPath_.push_back(ptr);
        ptr = ptr->cameFrom;
    }
    reverse(gridPath_.begin(),gridPath_.end());
    
    // cout <<"[debug] current target: "<< (*(gridPath_.end()-1))->coord<<endl;
    // cout <<"[debug] last ptr: "<< ptr->coord<<endl;
    GridNode2d real_start_pt;
    real_start_pt = *(*(gridPath_.begin()));
    real_start_pt.coord = start_pt;
    gridPath.push_back(&real_start_pt);
    auto start_pt_ = *(gridPath_.begin());
    while(!gridPath_.empty())
    {
        int id = 0, longest_id = 0;
        Eigen::Vector2i longest_colipoint(start_pt_->index);
        for(auto iter = gridPath_.begin(); iter != gridPath_.end(); iter++)
        {
            auto gridnode_ptr = *(iter);
            std::pair<bool,Eigen::Vector2i> check_point = check_collision(gridnode_ptr,start_pt_);
            if(!check_point.first)
                longest_id = id;
            id++;
        }
        cout<<"\033[0m"<<endl;
        // rate_.sleep();
        // cout << "[debug] current length: "<<gridPath_.size()<<endl;
        auto iter = gridPath_.begin();
        iter = iter + longest_id;
        cout << "[debug] current target coord: "<< (*(iter))->coord <<endl;
        gridPath.push_back(*(iter));
        start_pt_ = *(iter);
        gridPath_.erase(gridPath_.begin(),iter+1);
    }
    // cout <<"[debug] current set target: "<< (*(gridPath.end()-1))->coord<<endl;

    // 轨迹补全
    bool fk_flag = false;
    double last_remain_length = interval_;// 上一段轨迹残余长度

    auto iter_head = gridPath.begin();
    int j = 0;
    for(auto iter = gridPath.begin();iter!=gridPath.end();iter++)
    {
        auto start_ptr = *(iter_head);
        auto end_ptr   = *(iter);
        if((*iter_head)->coord == (*iter)->coord) continue;
        else
        {
            double   this_supp_length = interval_ - last_remain_length;
            Vector2d vector_s2e       = (end_ptr->coord - start_ptr->coord);  // 👈向量
            double lengh_of_line = vector_s2e.norm(); 
            if(lengh_of_line < this_supp_length)
            {
                last_remain_length = this_supp_length - lengh_of_line;
                continue;
            }
            Vector2d start_coord = start_ptr->coord +
                                   this_supp_length*vector_s2e/vector_s2e.norm();// 构建为当前位置加上一个初始向量长度
                       vector_s2e         = (end_ptr->coord - start_coord);
                       lengh_of_line      = vector_s2e.norm();                         // 向量长度
                   int count_insert       = (int)(lengh_of_line/(double)(interval_));
                       last_remain_length = lengh_of_line - interval_*count_insert;
            // cout << "\033[45m[debug] last_remain_length: " << last_remain_length << "\033[0m\n";

            for(int i = 0;i<=count_insert;i++)
            {
                Vector2d vector_online = start_coord + interval_*i*vector_s2e/vector_s2e.norm();
                path.push_back(vector_online);
            }
            j++;
            iter_head = iter;
        }
        if(iter == gridPath.end()-1)
        {
            path.push_back(end_ptr->coord);
        }
    }
    
    // if(!check_point.first)
    // {
    //     path.push_back((*(gridPath.end()-1))->coord);
    // }
    // else if(path.size() == 1) 
    // {
    //     path.push_back(*(path.end()-1));
    // }
    // cout << "\033[45m[debug] size of path: " << path.size() << "\033[0m\n";
    // path.push_back((*(gridPath.end()-1))->coord);
    return path;
}

std::pair<bool,Eigen::Vector2i> AstarPathFinder2d::check_collision(GridNodePtr2d aim_ptr,GridNodePtr2d start_ptr)
{
    Eigen::Vector2i start_idx = start_ptr->index;
    Eigen::Vector2i next_idx  = start_idx;
    Eigen::Vector2i aim_idx   = aim_ptr->index;
    Eigen::Vector2i step_;
    std::pair<bool,Eigen::Vector2i> end_idx(false, start_idx);
    int dx = abs(Eigen::Vector2i(aim_idx - start_idx).x()),          // 大小
        sx = Eigen::Vector2i(aim_idx - start_idx).x() > 0 ? 1 : -1;  // 符号
    int dy = abs(Eigen::Vector2i(aim_idx - start_idx).y()),
        sy = Eigen::Vector2i(aim_idx - start_idx).y() > 0 ? 1 : -1;
    int d = (dx > dy ? dx : -dy)/2;
    while (next_idx.x() != aim_idx.x() || next_idx.y() != aim_idx.y())
    {
        int d2 = d;
        if(d2 > -dx) {d -= dy; next_idx[0] += sx;}
        if(d2 <  dy) {d += dx; next_idx[1] += sy;}
        end_idx.second = next_idx;
        if(getData(next_idx))
        {
            cout << "\033[33m>";
            end_idx.first = true;
            break;
        }
    }
    if(!end_idx.first)
    {
        cout << "\033[37m>";
    }
    return end_idx;
    

}

std::pair<bool,Eigen::Vector2i> AstarPathFinder2d::check_collision(Eigen::Vector2d aim_ptr,Eigen::Vector2d start_ptr)
{
    Eigen::Vector2i start_idx = coord2gridIndex(start_ptr);
    Eigen::Vector2i next_idx  = start_idx;
    Eigen::Vector2i aim_idx   = coord2gridIndex(aim_ptr);
    Eigen::Vector2i step_;
    std::pair<bool,Eigen::Vector2i> end_idx(true, start_idx);
    int dx = abs(Eigen::Vector2i(aim_idx - start_idx).x()),          // 大小
        sx = Eigen::Vector2i(aim_idx - start_idx).x() > 0 ? 1 : -1;  // 符号
    int dy = abs(Eigen::Vector2i(aim_idx - start_idx).y()),
        sy = Eigen::Vector2i(aim_idx - start_idx).y() > 0 ? 1 : -1;
    int d = (dx > dy ? dx : -dy)/2;
    while (next_idx.x() != aim_idx.x() || next_idx.y() != aim_idx.y())
    {
        int d2 = d;
        if(d2 > -dx) {d -= dy; next_idx[0] += sx;}
        if(d2 <  dy) {d += dx; next_idx[1] += sy;}
        if(!getData(next_idx)) // 如果没发生碰撞
        {
            end_idx.first  = false;
            end_idx.second = next_idx;
        }
    }
    return end_idx;
    

}

// std::pair<bool,Eigen::Vector2i> AstarPathFinder2d::check_collision(GridNodePtr2d aim_ptr,GridNodePtr2d start_ptr)
// {
//     cout << "[CHECK] collision"<<endl;
//     Eigen::Vector2i start_idx = start_ptr->index;
//     Eigen::Vector2i aim_idx   = aim_ptr->index;
//     Eigen::Vector2i check_idx = aim_idx;
//     bool collision_happened = false;
//     std::pair<bool,Eigen::Vector2i> end_idx(collision_happened, start_idx);

//     int delta_x_ = abs(Eigen::Vector2i(aim_idx - start_idx).x()),
//         sx = Eigen::Vector2i(aim_idx - start_idx).x() > 0 ? 1 : -1;
//     int delta_y_ = abs(Eigen::Vector2i(aim_idx - start_idx).y()),
//         sy = Eigen::Vector2i(aim_idx - start_idx).y() > 0 ? 1 : -1;
//     double slope_ = double(delta_y_)/double(delta_x_);
//     int erro = (delta_x_ > delta_y_ ? delta_x_ : -delta_y_) / 2;

//     Eigen::Vector2i next_idx = check_idx;
//     ros::Rate rsss(1.0);
//     while(next_idx.x() != start_idx.x() && next_idx.y() != start_idx.y()) // 到起点截止
//     {
//         rsss.sleep();
//         int e2 = erro;
//         if(e2 > -delta_x_) { erro -= delta_y_; next_idx[0] += sx;}
//         if(e2 <  delta_y_) { erro += delta_x_; next_idx[1] += sy;}
//         cout << "next_idx" <<next_idx << endl;
//         if(!getData(next_idx))// 如果没发生碰撞
//         {
//             check_idx = next_idx;
//         }
//         else
//         {
//             ROS_WARN("collision_happened");
//             collision_happened = true;
//             break;
//         }
//     }
//     end_idx = std::pair<bool,Eigen::Vector2i>(collision_happened, check_idx);
//     return end_idx;

// }

//TODO 反向追溯得到路径
vector<Vector2d> AstarPathFinder2d::getPath() 
{   
    vector<Vector2d> path;
    vector<GridNodePtr2d> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    auto ptr = terminatePtr;
    while(ptr -> cameFrom != NULL){
        gridPath.push_back(ptr);
        ptr = ptr->cameFrom;
    }
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());
    return path;
}

// if the difference of f is trivial, then choose then prefer the path along the straight line from start to goal
// discared!!!
// 目前这个函数我没有用到.
GridNodePtr2d & TieBreaker(const std::multimap<double, GridNodePtr2d> &  openSet, const GridNodePtr2d & endPtr)
{
    // todo do I have to update the f in openSet??
    std::multimap<double, GridNodePtr2d> local_set;

    auto f_min = openSet.begin()->first;
    auto f_max = f_min + 1e-2;
    auto itlow = openSet.lower_bound (f_min);
    auto itup = openSet.upper_bound(f_max);
    double cross, f_new;

    for (auto it=itlow; it!=itup; ++it)
    {
        std::cout << "f value is:" << (*it).first << " pointer is: " << (*it).second << '\n';
        cross = std::abs(endPtr->coord(0) - (*it).second->coord(0)) +
                std::abs(endPtr->coord(1) - (*it).second->coord(1));
        f_new = (*it).second->fScore + 0.001 * cross;
        local_set.insert( make_pair(f_new, (*it).second) ); // todo what is iterator, is this way correct?
    }


    return local_set.begin()->second;
}

int AstarPathFinder2d::twistTest(int i,int j){
    return 3*(i+1)+(j+1)+1;
}