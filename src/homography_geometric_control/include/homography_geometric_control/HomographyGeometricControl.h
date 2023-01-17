#ifndef __HOMOGRAQPHY_GEOMETRIC_CONTROL__
#define __HOMOGRAQPHY_GEOMETRIC_CONTROL__

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>


#include <uav_state/UAVState.h>
#include <homo_msgs/HomographyResult.h>


namespace HomographyGeometricControl
{

class HomographyGeometric
{
public:
struct ControlGain
{
    /* data */
    double c;
    Eigen::Matrix3d Kv;

    double k1,k2,k3;
};
private:
    /* data */
    ros::NodeHandle nh_;
    ros::NodeHandle nhParam_;
    ros::Subscriber homographySub;

    ControlGain controlGain_;
    Eigen::Matrix3d homography_,homographyVirtual_;
    double controlRate_;
    Control::Quadrotor curUavState_;//当前的无人机状态 basened相对refned

    /*    curUavState_ 内部变量
    *Eigen::Vector3d pos;//位置 ->
    *Eigen::Vector3d vel;//速度 ->需要相对于机体nedzuo坐标系下的速度
    *Eigen::Vector3d acc;//加速度 ->
    *Eigen::Vector3d jerk;//加加速度 ->
    *Eigen::Vector3d eulerAngle;//角度  欧拉角
    *Eigen::Vector3d omega;//角速度 默认为相对于机体nedzuo坐标系下的速度
    *Eigen::Matrix3d R;//旋转矩阵->需要相对于世界nedzuo坐标系下的旋转矩阵
    *Eigen::Quaterniond orientation;//四元数 ->
    *double mass;//质量
    *double g;//重力加速度大小
    *Eigen::Matrix3d J;//惯性矩阵；
     */
    Eigen::Matrix3d RZ_,RY_,RX_;//当前欧拉角各自的旋转矩阵
    double yawFromHomographyVirtual_;//从虚拟单应性中求解的偏航角
    Eigen::Matrix3d RZFromHomographyVirtual_;//从yawFromHomographyVirtual_构建的偏航旋转矩阵
    Eigen::Vector3d e1_,e2_;
    Eigen::Vector3d FVitual_;
    Eigen::Matrix3d RDesired_;
    Eigen::Matrix3d dot_RDesired_;
    Eigen::Vector3d omegaDesired_;
    double thrust_;//论文里为正数，为电机推力之和

    //以下变量为常量
    Eigen::Vector3d mStar_;
    Eigen::Vector3d axisZ_;
    double yawDesired_,yawDesiredLast_;//期望偏航角
    double dot_yawDesired_;//期望偏航角角速度
    Eigen::Vector3d b1c_;//
    double thrustOffest_;//重力和推力平衡时的零偏值 0～1
    double thrustScale_;//推力的缩放比例
    Eigen::Vector3d eulerAngleOffest_;

    //速度观测器相关变量
    Eigen::Vector3d  velVitualEstToTrue_;//估计的虚拟框架速度
    Eigen::Vector3d  e1Error_;
    Eigen::Vector3d  e1Est_;
    Eigen::Vector3d  velVitualError_;
    Eigen::Vector3d  velVitualEst_;//估计的虚拟框架速度
    bool homographyCallbackState;

    virtual void HomographyCallback(const homo_msgs::HomographyResult::ConstPtr& msg);

private:
    const double UpdateyawFromHomographyVirtual();
    const Eigen::Vector3d UpdateError1();
    const Eigen::Vector3d UpdateError2FromTrueVel();
    const Eigen::Vector3d UpdateError2FromEstVel();
    const Eigen::Vector3d UpdateFVitual();
    const double UpdateThrust();
    const Eigen::Matrix3d UpdateRotationDesired();
    const Eigen::Vector3d UpdateOmegaDesired();
    const Eigen::Vector3d UpdateVelocityEstimation();

public:
    HomographyGeometric(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~HomographyGeometric();

    void operator() (const Control::Quadrotor& curUavState);
    void SetCurUavState(const Control::Quadrotor& val);


    const Control::Quadrotor& GetQuadrotor()const;
    const Eigen::Matrix3d& GetRDesired()const;
    const Eigen::Vector3d& GetOmegaDesired()const;
    const double& GetThrust()const;
    const double& GetControlRate() const;
    const Eigen::Vector3d& GetE1() const;
    const Eigen::Vector3d& GetE2() const;

    const Eigen::Vector3d& GetE1Error() const;
    const Eigen::Vector3d& GetE1Est() const;
    const Eigen::Vector3d& GetVelVitualError() const;
    const Eigen::Vector3d& GetVelVitualEst() const;
    const Eigen::Vector3d GetVelVitualTure() const;

    void ShowInternal(int num = 5) const;
    void ShowParamVal(int num = 5) const;
};









}
#endif