#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>


#include <ros/ros.h>

#include <uav_state/UAVState.h>

int main(int argc, char *argv[]){

    ros::init(argc, argv, "so3_control_test");
    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(60.0);
    Eigen::Matrix3d J;
    J << 1,0,0,0,1,0,0,0,1;
    double m = 1.5;
    double g = 9.8091;
    UAVState::Quadrotor test;
    while(ros::ok()){
        std::cout<<test.GetGravity()<<std::endl;
        std::cout<<test.GetMass()<<std::endl;
        std::cout<<test.GetInertialMatrix()<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}