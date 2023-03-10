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
    nhParam_.param("ControlGain/c2",controlGain_.c2,0.15);
    nhParam_.param("ControlGain/c3",controlGain_.c3,1.0);
    nhParam_.param("ControlGain/Ks",controlGain_.Ks,1.0);
    controlGain_.Kp2 =  Eigen::Matrix3d::Identity();
    nhParam_.param("ControlGain/Kp2x",controlGain_.Kp2(0,0),1.0);
    nhParam_.param("ControlGain/Kp2y",controlGain_.Kp2(1,1),1.0);
    nhParam_.param("ControlGain/Kp2z",controlGain_.Kp2(2,2),1.0);
    omegaDesired_ = Eigen::Vector3d::Zero();
    RDesired_ = Eigen::Matrix3d::Identity();
    torque_ = Eigen::Vector3d::Zero();
    eR_ = Eigen::Vector3d::Zero();
    eOmega_ = Eigen::Vector3d::Zero();
    attitudeTrackError_ = 0.0;
    momentEst_ = Eigen::Vector3d::Zero();
    momentTmp_ = Eigen::Vector3d::Zero();
    nhParam_.param("MomentDisturbanceEstimatorEnable",MomentDisturbanceEstimatorEnable,0); 
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
    Eigen::Matrix3d RTmp;
    RTmp = RDesired_.transpose()*curUavState_.GetR();
    res = Common::MatrixHatInv(Common::LogMatrix(RTmp));
    // res = 0.5*Common::MatrixHatInv(RDesired_.transpose()*curUavState_.GetR() 
    //                         - curUavState_.GetR().transpose()*RDesired_);
    return res;
}

const Eigen::Vector3d 
SO3Control::UpdateEOmega()
{
    Eigen::Vector3d res;
    res = curUavState_.GetOmega() - RDesired_.transpose()*curUavState_.GetR()*omegaDesired_;
    return res;
}

const Eigen::Vector3d 
SO3Control::UpdateTorque()
{
    Eigen::Vector3d res,s;
    Eigen::Vector3d part1,part2,part3;
    static double dot_Ks = 0;
    s = controlGain_.KR * eR_ + eOmega_;
    Common::ShowVal("dot_Ks",dot_Ks,5);
    controlGain_.Ks += dot_Ks/controlRate_;
    controlGain_.Ks = std::max(-0.03,std::min(0.03,controlGain_.Ks));
    part1 = Common::MatrixHat(curUavState_.GetOmega()) * curUavState_.GetInertialMatrix() * curUavState_.GetOmega();
    part2 = - controlGain_.Ks*Common::Sign(s);
    part3 = - controlGain_.c2*s;
    
    // res = part3 +  part2 + part1 ;
    res =  part3 + part1 ;

    dot_Ks = - controlGain_.c3* controlGain_.Ks + 0.1*s.transpose()*Common::Sign(s);
    return res;
}

const double 
SO3Control::UpdateAttitudeTrackError()
{
    double res;
    res = 0.5*(Eigen::Matrix3d::Identity() - RDesired_.transpose() * curUavState_.GetR()).trace();
    return res;
}

const Eigen::Vector3d 
SO3Control::UpdateMomentEstimation()
{
    Eigen::Vector3d res;
    Eigen::Matrix3d A,D;
    Eigen::Vector3d B,C;
    Eigen::Vector3d dot_momentTmp = Eigen::Vector3d::Zero();
    A = curUavState_.GetInertialMatrix();
    B = Common::MatrixHat(curUavState_.GetOmega()) * curUavState_.GetInertialMatrix() * curUavState_.GetOmega();
    C = Eigen::Vector3d::Zero();
    D = - Eigen::Matrix3d::Identity();
    dot_momentTmp = - controlGain_.Kp2*momentTmp_
                    + controlGain_.Kp2*(B + C + D*torque_ - A*curUavState_.GetOmega());
    momentTmp_ = momentTmp_ + dot_momentTmp/controlRate_;
    res = momentTmp_ + controlGain_.Kp2*A*curUavState_.GetOmega();
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

    if(MomentDisturbanceEstimatorEnable == 2){
        torque_ = UpdateTorque() - momentEst_;
    }else {
        torque_ = UpdateTorque();
    }
    
    attitudeTrackError_ = UpdateAttitudeTrackError();
    
    if(MomentDisturbanceEstimatorEnable != 0){
        momentEst_ = UpdateMomentEstimation();
    }
    ShowInternal(5);
    //ShowParamVal(5);
}


void 
SO3Control::ShowInternal(int num) const
{
    std::cout << "-------------------SO3Control-------------------" <<std::endl;
    Common::ShowVal("eR_",eR_,num);
    Common::ShowVal("eOmega_",eOmega_,num);
    Common::ShowVal("torque_",torque_,num);
    Common::ShowVal("momentEst_",momentEst_,num);
}

void 
SO3Control::ShowParamVal(int num) const
{
    Common::ShowVal("controlRate_",controlRate_,num);
    Common::ShowVal("controlGain_.KR",controlGain_.KR,num);
    Common::ShowVal("controlGain_.KOmega",controlGain_.KOmega,num);
    Common::ShowVal("controlGain_.c2",controlGain_.c2,num);
    Common::ShowVal("controlGain_.c3",controlGain_.c3,num);
    Common::ShowVal("controlGain_.Ks",controlGain_.Ks,num);
}
} // namespace Control
