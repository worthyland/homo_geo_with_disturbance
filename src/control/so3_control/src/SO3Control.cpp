#include <so3_control/SO3Control.h>


namespace Control
{
SO3Control::SO3Control(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam):nhParam_(nhParam)
{
    nhParam_.param("Control_rate",controlRate_,200.0);
    controlGain_.KR = Eigen::Matrix3d::Identity();
    nhParam_.param("ControlGain/KRx",controlGain_.KR(0,0),0.1);
    nhParam_.param("ControlGain/KRy",controlGain_.KR(1,1),0.1);
    nhParam_.param("ControlGain/KRz",controlGain_.KR(2,2),0.1);
    controlGain_.KOmega = Eigen::Matrix3d::Identity();
    nhParam_.param("ControlGain/KOmegax",controlGain_.KOmega(0,0),0.01);
    nhParam_.param("ControlGain/KOmegay",controlGain_.KOmega(1,1),0.01);
    nhParam_.param("ControlGain/KOmegaz",controlGain_.KOmega(2,2),0.01);
    attitudeTrackError_ = 0.0;
}

SO3Control::~SO3Control()
{
}


void 
SO3Control::SetState(const Quadrotor& val)
{
    curUavState_ = val;
}

const double& 
SO3Control::GetControlRate() const
{
    return controlRate_;
}

void 
SO3Control::SetRDesired(const Eigen::Matrix3d& val)
{
    RDesired_ = val;
}

const Eigen::Matrix3d& 
SO3Control::GetRDesired() const
{
    return RDesired_;
}

void 
SO3Control::SetOmegaDesired(const Eigen::Vector3d& val)
{
    omegaDesired_ = val;
}
const Eigen::Vector3d& 
SO3Control::GetOmegaDesired() const
{
    return omegaDesired_;
}

const Eigen::Vector3d& 
SO3Control::GetTorque() const
{
    return torque_;
}

const Eigen::Vector3d& 
SO3Control::GetER() const
{
    return eR_;
}

const Eigen::Vector3d& 
SO3Control::GetEOmega() const
{
    return eOmega_;
}

const double& 
SO3Control::GetAttitudeTrackError() const
{
    return attitudeTrackError_;
}

const Eigen::Vector3d 
SO3Control::UpdateER()
{
    Eigen::Vector3d res;
    res = 0.5*Common::MatrixHatInv(RDesired_.transpose()*curUavState_.GetR() 
                            - curUavState_.GetR().transpose()*RDesired_);
    return res;
}

const Eigen::Vector3d 
SO3Control::UpdateEOmega()
{
    Eigen::Vector3d res;
    res = curUavState_.GetOmega() - curUavState_.GetR()*RDesired_*omegaDesired_;
    return res;
}

const Eigen::Vector3d 
SO3Control::UpdateTorque()
{
    Eigen::Vector3d res;
    Eigen::Vector3d part1,part2,part3;
    part1 = Common::MatrixHat(curUavState_.GetOmega()) * curUavState_.GetInertialMatrix() * curUavState_.GetOmega();
    part2 = - controlGain_.KR * eR_;
    part3 = - controlGain_.KOmega * eOmega_;
    res = part1 + part2 + part3;
    // res = part2 + part3;
    return res;
}
const double 
SO3Control::UpdateAttitudeTrackError()
{
    double res;
    res = 0.5*(Eigen::Matrix3d::Identity() - RDesired_.transpose() * curUavState_.GetR()).trace();
    return res;
}
void 
SO3Control::operator() (const Eigen::Matrix3d& RDesired,const Eigen::Vector3d& omegaDesired,const Control::Quadrotor& curUavState)
{
    SetRDesired(RDesired);
    SetOmegaDesired(omegaDesired);
    SetState(curUavState);
    
    eR_ = UpdateER();
    eOmega_ = UpdateEOmega();
    torque_ = UpdateTorque();
    attitudeTrackError_ = UpdateAttitudeTrackError();
    
    ShowInternal(5);
    // ShowParamVal(5);
}


void 
SO3Control::ShowInternal(int num) const
{
    std::cout << "-------------------SO3Control-------------------" <<std::endl;
    Common::ShowVal("eR_",eR_,num);
    Common::ShowVal("eOmega_",eOmega_,num);
    Common::ShowVal("torque_",torque_,num);

}

void 
SO3Control::ShowParamVal(int num) const
{
    Common::ShowVal("controlRate_",controlRate_,num);
    Common::ShowVal("controlGain_.KR",controlGain_.KR,num);
    Common::ShowVal("controlGain_.KOmega",controlGain_.KOmega,num);
}
} // namespace Control
