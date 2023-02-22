/*

*/

#ifndef __MAVROS_INTERACTION__
#define __MAVROS_INTERACTION__
//系统的头文件
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>

#include <std_msgs/Float32MultiArray.h>
//需要的头文件
#include <uav_state/UAVState.h>
namespace Control
{

class MavrosInteraction
{
private:
    /* data */
    ros::NodeHandle nh_;
    ros::NodeHandle nhParam_;
    /* 话题订阅 */
    ros::Subscriber posSub_;//位置信息 世界框架下z轴向上为正 
    ros::Subscriber imuSub_;//获取IMU的信息 base_link 话题名称：mavros/imu/data 更新角速度（相对于机体坐标系）/加速度（相对于机体坐标系  转化为相对世界坐标系）（Z轴为9.8，加速度的值）/    四元数 相对于世界坐标系
    ros::Subscriber velocityBodySub_;//相对于机体坐标下速度,角速度 机体框架z轴向上为正 base_link 话题名称：mavros/local_position/velocity_body 
    ros::Subscriber velocityLocalSub_;//相对与世界坐标下速度，角速度 世界框架z轴向上为正 base_link 话题名称：mavros/local_position/velocity_local
    ros::Subscriber px4ControlStateSub_;//mavros/state
    

    /* 话题发布 */
    ros::Publisher mixPub_;//力矩发布控制
    double minTorque_,maxTorque_;//输出力矩限制
    double minThrust_,maxThrust_;//输出推力限制
    /* 服务端*/
    ros::ServiceClient armingClient;
    ros::ServiceClient setModeClient;

    bool posCallbackState,imuCallbackState,velocityBodyCallbackState,velocityLocalCallbackState,offboardState; 

    Quadrotor uav_;
    mavros_msgs::State currentControlState_;

    

private:
    virtual void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    virtual void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
    virtual void VelocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    virtual void VelocityLocalCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    virtual void PX4ControlStateCallback(const mavros_msgs::State::ConstPtr& msg);

public:
    MavrosInteraction();
    MavrosInteraction(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~MavrosInteraction();

    void ShowUavState(int num) const;

    const Quadrotor::state& GetState()const;
    const Quadrotor& GetQuadrotor()const;
    const mavros_msgs::State& GetCurrentControlState()const;

    void ActuatorPub(const double& thrust = 0.50,const Eigen::Vector3d& torque = {0,0,0},bool showVal=false) const;//推力 力矩发布

    void DataSvae(std_msgs::Float32MultiArray& outputRecord,const Eigen::Vector3d& val) const;//数据发布中转函数
    void DataSvae(std_msgs::Float32MultiArray& outputRecord,const double& val) const;//数据发布中转函数
};




}
#endif