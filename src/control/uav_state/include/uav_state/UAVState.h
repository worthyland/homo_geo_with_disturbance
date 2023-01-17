#ifndef __UAV_STATE__
#define __UAV_STATE__

#include <Eigen/Core>
#include <Eigen/Dense>
#include<iostream>
#include<iomanip>
#include <uav_state/Common.h>
namespace Control
{


class Quadrotor
{
public:
struct state
{
    /* data */
    Eigen::Vector3d pos;//位置
    Eigen::Vector3d vel;//速度
    Eigen::Vector3d acc;//加速度
    Eigen::Vector3d jerk;//加加速度
    
    Eigen::Vector3d eulerAngle;//角度  欧拉角
    Eigen::Vector3d omega;//角速度 
    Eigen::Matrix3d R;//旋转矩阵
    
    Eigen::Quaterniond orientation;//四元数
    double mass;//质量
    double g;//重力加速度
    Eigen::Matrix3d J;//惯性矩阵；
};
private:
    state state_;
public:    
    Quadrotor();
    Quadrotor(const double& mass_,const double& g_,const Eigen::Matrix3d& J_);
    ~Quadrotor();
    void UpdateState(void);
    void SetMass(const double& val);
    const double& GetMass(void) const;
    void SetGravity(const double& val);
    const double& GetGravity(void) const;
    void SetInertialMatrix(const Eigen::Matrix3d& val);
    const Eigen::Matrix3d& GetInertialMatrix(void)const;
    void SetPos(const Eigen::Vector3d& val);
    const Eigen::Vector3d& GetPos(void) const;
    void SetOmega(const Eigen::Vector3d& val);
    const Eigen::Vector3d& GetOmega(void) const;
    void SetAcc(const Eigen::Vector3d& val);
    const Eigen::Vector3d& GetAcc(void) const;
    void SetOrientation(const Eigen::Quaterniond& val);
    const Eigen::Quaterniond& GetOrientation(void) const;
    void SetVel(const Eigen::Vector3d& val);
    const Eigen::Vector3d& GetVel(void) const;
    void SetR(const Eigen::Matrix3d& val);//
    const Eigen::Matrix3d& GetR(void) const;
    void SetEulerAngle(const Eigen::Vector3d& val);//
    const Eigen::Vector3d& GetEulerAngle(void) const;

    const Quadrotor::state& GetState(void)const;
    
    const Eigen::Vector3d QuaternionToEulerAngles(const Eigen::Quaterniond& q) const;//四元数转欧拉角 zyx Eigen库的函数有点问题
    void RotationFrameTransform(const Eigen::Matrix3d& Left,const Eigen::Matrix3d& Right);//将世界框架的state状态量进行变换，变换到ned坐标系下
    void Vector3dFrameTransform(const Eigen::Matrix3d& Left);


    void ShowState(const std::string& str="",int num=5) const;//显示到小数点后5位  由num设置
    // void ShowVal(const std::string& str,const Eigen::Vector3d& val,int num = 5) const;
    // void ShowVal(const std::string& str,const Eigen::Matrix3d& val,int num = 5) const;
    // void ShowVal(const std::string& str,const Eigen::Quaterniond& val,int num = 5) const;
    // void ShowVal(const std::string& str,const double& val,int num = 5) const;    
};






}

#endif