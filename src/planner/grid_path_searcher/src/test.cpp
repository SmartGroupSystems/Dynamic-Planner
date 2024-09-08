#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/TwistStamped.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <complex.h>
using namespace std;

int main(){
    vector<int> myvector;
    for(int i=0;i<2;i++)
        {
            myvector.push_back(i);
        }
        cout << myvector.size()<<endl;
        cout <<"======================"<<endl;
        for(auto it=myvector.begin();it!=myvector.end();it++)
        cout<<*it<<endl;

        cout <<"======================"<<endl;
    myvector.erase(myvector.begin(),myvector.end());//删除前3个元素,[ .. ),第四个元素没有删除
    for(auto it=myvector.begin();it!=myvector.end();it++)
        cout<<*it<<endl;
        cout <<"======================"<<endl;
        cout << myvector.size()<<endl;
    return 0;
}