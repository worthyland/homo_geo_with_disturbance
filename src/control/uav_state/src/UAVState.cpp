#include <iostream>
#include <uav_state/UAVState.h>

namespace Control
{

Quadrotor::Quadrotor(const double& mass_,const double& g_,const Eigen::Matrix3d& J_)
{
    state_.mass = mass_;
    state_.g = g_;
    state_.pos = Eigen::Vector3d::Zero();
    state_.vel = Eigen::Vector3d::Zero();
    state_.acc = Eigen::Vector3d::Zero();
    state_.jerk = Eigen::Vector3d::Zero();
    state_.eulerAngle = Eigen::Vector3d::Zero();
    state_.omega = Eigen::Vector3d::Zero();
    state_.R = Eigen::Matrix3d::Identity();
    state_.J = J_;
    Eigen::Quaterniond orientationTmp(1,0,0,0);
    state_.orientation = orientationTmp;
}

Quadrotor::Quadrotor()
{
    state_.mass = 1.00;
    state_.g = 9.80665;
    state_.pos = Eigen::Vector3d::Zero();
    state_.vel = Eigen::Vector3d::Zero();
    state_.acc = Eigen::Vector3d::Zero();
    state_.jerk = Eigen::Vector3d::Zero();
    state_.eulerAngle = Eigen::Vector3d::Zero();
    state_.omega = Eigen::Vector3d::Zero();
    state_.R = Eigen::Matrix3d::Identity();
    state_.J = Eigen::Matrix3d::Identity() * 0.01;
    Eigen::Quaterniond orientationTmp(1,0,0,0);
    state_.orientation = orientationTmp;
}

Quadrotor::~Quadrotor()
{
}


void 
Quadrotor::UpdateState()
{

}

void 
Quadrotor::SetMass(const double& val)
{
    state_.mass=val;
}

const double&
Quadrotor::GetMass(void) const
{
    return state_.mass;
}

void 
Quadrotor::SetGravity(const double& val)
{
    state_.g = val;
}

const double&
Quadrotor::GetGravity(void) const
{
    return state_.g;
}

void 
Quadrotor::SetInertialMatrix(const Eigen::Matrix3d& val)
{
    state_.J = val;
}

const Eigen::Matrix3d& 
Quadrotor::GetInertialMatrix(void)const
{
    return state_.J;
}

void 
Quadrotor::SetPos(const Eigen::Vector3d& val)
{
    state_.pos = val;
}

const Eigen::Vector3d& 
Quadrotor::GetPos(void)const
{
    return state_.pos;
}

void 
Quadrotor::SetOmega(const Eigen::Vector3d& val)
{
    state_.omega = val;
}

const Eigen::Vector3d& 
Quadrotor::GetOmega(void) const
{
    return state_.omega;
}


void 
Quadrotor::SetAcc(const Eigen::Vector3d& val)
{
    state_.acc = val;
}

const Eigen::Vector3d& 
Quadrotor::GetAcc(void) const
{
    return state_.acc;
}

void 
Quadrotor::SetOrientation(const Eigen::Quaterniond& val)
{
    state_.orientation = val;
}

const Eigen::Quaterniond& 
Quadrotor::GetOrientation(void) const
{
    return state_.orientation;
}

void 
Quadrotor::SetVel(const Eigen::Vector3d& val)
{
    state_.vel = val;
}

const Eigen::Vector3d& 
Quadrotor::GetVel(void) const
{
    return state_.vel;
}

void 
Quadrotor::SetR(const Eigen::Matrix3d& val)
{
    state_.R = val;
}

const Eigen::Matrix3d& 
Quadrotor::GetR(void) const
{
    return state_.R;
}

void 
Quadrotor::SetEulerAngle(const Eigen::Vector3d& val)
{
    state_.eulerAngle = val;
}
const Eigen::Vector3d& 
Quadrotor::GetEulerAngle(void) const
{
    return state_.eulerAngle;
}

const Quadrotor::state& 
Quadrotor::GetState(void) const
{
    return state_;
}


const Eigen::Vector3d
Quadrotor::QuaternionToEulerAngles(const Eigen::Quaterniond& q) const
{

    Eigen::Vector3d angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2) = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0) = std::atan2(siny_cosp, cosy_cosp);
 

    return angles;
}

/**
 * @brief 转化state 具体为  Left * （state） * Right 将旋转矩阵进行坐标系变换
 *  主要改变 R orientation eulerAngle
 * @param Left 左乘的旋转矩阵
 * @param Right 右乘的旋转矩阵
 */
void 
Quadrotor::RotationFrameTransform(const Eigen::Matrix3d& Left,const Eigen::Matrix3d& Right)
{
    state_.R = Left * state_.R * Right;
    state_.orientation = state_.R;
    state_.eulerAngle = QuaternionToEulerAngles(state_.orientation);
}

/**
 * @brief 将内部vector3d向量变换坐标系
 *  R1 * （vector3d）
 * @param Left 乘的旋转矩阵
 */
void 
Quadrotor::Vector3dFrameTransform(const Eigen::Matrix3d& Left)
{
    state_.pos = Left * state_.pos;
    state_.vel = Left * state_.vel;
    state_.acc = Left * state_.acc;
    state_.jerk = Left * state_.jerk;
}
/**
 * @brief 显示状态量
 * 
 * @param num 控制显示的小数点后位数
 */
void Quadrotor::ShowState(const std::string& str,int num) const
{
    std::cout << "---------------"<<str<<":状态量显示"<<"---------------- " <<std::endl;
    Common::ShowVal("质量mass",state_.mass,num);
    Common::ShowVal("重力加速度g",state_.g,num);
    Common::ShowVal("位置pos",state_.pos,num);
    Common::ShowVal("速度vel",state_.vel,num);
    Common::ShowVal("加速度acc",state_.acc,num);
    Common::ShowVal("加加速度jerk",state_.jerk,num);
    Common::ShowVal("欧拉角eulerAngle",state_.eulerAngle,num);
    Common::ShowVal("角速度omega",state_.omega,num);
    Common::ShowVal("旋转矩阵R",state_.R,num);
    Common::ShowVal("四元数orientation",state_.orientation,num);

}


// void 
// Quadrotor::ShowVal(const std::string& str,const Eigen::Vector3d& val,int num) const
// {
//     std::cout << str <<":";
//     std::cout<<std::fixed<< std::setprecision(num)<< val(0)<<","<<val(1)<<","<<val(2)<<","<<std::endl;
// }

// void 
// Quadrotor::ShowVal(const std::string& str,const Eigen::Matrix3d& val,int num) const
// {
//     std::cout << str << ":" << std::endl;
//     std::cout<<std::fixed<< std::setprecision(num)<< val<<std::endl;
// }

// void 
// Quadrotor::ShowVal(const std::string& str,const Eigen::Quaterniond& val,int num) const
// {
//     std::cout << str <<":";
//     std::cout<<std::fixed<< std::setprecision(num)<< val.w()<<","<<val.x()<<","
//                                                 <<val.y()<<","<<val.z()<<std::endl;
// }

// void 
// Quadrotor::ShowVal(const std::string& str,const double& val,int num) const
// {
//     std::cout << str <<":";
//     std::cout<<std::fixed<< std::setprecision(num)<< val <<std::endl;
// }
}
