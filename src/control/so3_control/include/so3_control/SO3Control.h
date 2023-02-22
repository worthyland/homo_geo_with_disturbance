#ifndef __SO_CONTROL__
#define __SO_CONTROL__

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>


#include <uav_state/UAVState.h>
namespace Control
{

class SO3Control
{
public:
struct ControlGain
{
    /* data */
    Eigen::Matrix3d KR;
    Eigen::Matrix3d KOmega;
    double c2,c3;
    double Ks;
    Eigen::Matrix3d Kp2;
};

private:
    /* data */
    ros::NodeHandle nh_;
    ros::NodeHandle nhParam_;
    double controlRate_;
    ControlGain controlGain_;
    Quadrotor curUavState_;

    Eigen::Vector3d omegaDesired_;
    Eigen::Matrix3d RDesired_;
    Eigen::Vector3d torque_;
    Eigen::Vector3d eR_,eOmega_;//姿态误差

    double attitudeTrackError_;


    //外力矩扰动观测器相关变量
    Eigen::Vector3d momentEst_;//作用于机体框架的外力矩估计值
    Eigen::Vector3d momentTmp_;//中间变量
    int MomentDisturbanceEstimatorEnable;//扰动估计器 0 不使能  1 使能但不使用 2 使能且使用

private:
    const Eigen::Vector3d UpdateER();
    const Eigen::Vector3d UpdateEOmega();
    const Eigen::Vector3d UpdateTorque();
    const double UpdateAttitudeTrackError();
    const Eigen::Vector3d UpdateMomentEstimation();
public:
    SO3Control(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~SO3Control();

    void SetState(const Quadrotor& val);
    const double& GetControlRate() const;
    void SetRDesired(const Eigen::Matrix3d& val);
    const Eigen::Matrix3d& GetRDesired() const;
    void SetOmegaDesired(const Eigen::Vector3d& val);
    const Eigen::Vector3d& GetOmegaDesired() const;

    const Eigen::Vector3d& GetTorque() const;
    const Eigen::Vector3d& GetER() const;
    const Eigen::Vector3d& GetEOmega() const;
    const double& GetAttitudeTrackError() const;
    const Eigen::Vector3d& GetMomentEst() const;

    void operator() (const Eigen::Matrix3d& RDesired,const Eigen::Vector3d& omegaDesired,const Control::Quadrotor& curUavState);
    void ShowInternal(int num = 5) const;
    void ShowParamVal(int num = 5) const;
};

} // namespace CONTROL

#endif