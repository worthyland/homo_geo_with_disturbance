#include <homography_geometric_control/HomographyGeometricControl.h>

namespace HomographyGeometricControl
{

HomographyGeometric::HomographyGeometric(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam):nh_(nh),nhParam_(nhParam)
{
    homographySub = nh_.subscribe("homography_pose",1,&HomographyGeometric::HomographyCallback,this);

    //参数初始化
    nhParam_.param("Control_rate",controlRate_,200.0);
    nhParam_.param("ControlGain/c",controlGain_.c,1.0);
    nhParam_.param("YawDesired",yawDesired_,0.0);
    b1c_ << cos(yawDesired_),sin(yawDesired_),0;
    dot_yawDesired_ = 0.0;
    controlGain_.Kv = Eigen::Matrix3d::Identity();
    nhParam_.param("ControlGain/Kvx",controlGain_.Kv(0,0),1.0);
    nhParam_.param("ControlGain/Kvy",controlGain_.Kv(1,1),1.0);
    nhParam_.param("ControlGain/Kvz",controlGain_.Kv(2,2),1.0);
    nhParam_.param("ThrustOffest",thrustOffest_,0.40);
    nhParam_.param("ThrustScale",thrustScale_,0.025);
    nhParam_.param("ControlGain/k1",controlGain_.k1,5.0);
    nhParam_.param("ControlGain/k2",controlGain_.k2,1.0);
    nhParam_.param("ControlGain/k3",controlGain_.k3,1.0);
    nhParam_.param("YawOffest",eulerAngleOffest_(2),0.0);
    nhParam_.param("RollOffest",eulerAngleOffest_(1),0.0);
    nhParam_.param("PitchOffest",eulerAngleOffest_(0),0.0);
    homography_ = Eigen::Matrix3d::Identity();
    homographyVirtual_ = Eigen::Matrix3d::Identity();
    RZ_ = Eigen::Matrix3d::Identity();
    RY_ = Eigen::Matrix3d::Identity();
    RX_ = Eigen::Matrix3d::Identity();
    e1_ = Eigen::Vector3d::Zero();
    e2_ = Eigen::Vector3d::Zero();
    FVitual_ = Eigen::Vector3d::Zero();
    RDesired_ = Eigen::Matrix3d::Identity();
    dot_RDesired_ = Eigen::Matrix3d::Zero();
    omegaDesired_ = Eigen::Vector3d::Zero();
    yawFromHomographyVirtual_ = 0.0;
    RZFromHomographyVirtual_ = Eigen::Matrix3d::Identity();
    mStar_ << 0,0,1;
    axisZ_ << 0,0,1;
    thrust_ = curUavState_.GetMass() * curUavState_.GetGravity();

    homographyCallbackState = false;
    
}

HomographyGeometric::~HomographyGeometric()
{
}


void 
HomographyGeometric::HomographyCallback(const homo_msgs::HomographyResult::ConstPtr& msg)
{
    homographyCallbackState = true;
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            homography_(i,j) = msg->homography[i*3+j];
        }
    }
    homographyVirtual_ = RY_ * RX_ * homography_;
    // std::cout << "homography_:" << std::endl;
    //Common::ShowVal("homographyVirtual_:",homographyVirtual_);
}

void 
HomographyGeometric::SetCurUavState(const Control::Quadrotor& val)
{
    curUavState_ = val;
}

const double 
HomographyGeometric::UpdateyawFromHomographyVirtual()
{
    double res;
    res = asin(0.5*Common::MatrixHatInv(homographyVirtual_.transpose() - homographyVirtual_).transpose() * axisZ_);
    return res;
}
const Eigen::Vector3d
HomographyGeometric::UpdateError1()
{
    Eigen::Vector3d res;
    res = (Eigen::Matrix3d::Identity() - homographyVirtual_) * mStar_;
    return res;
}

const Eigen::Vector3d
HomographyGeometric::UpdateError2FromTrueVel()
{
    Eigen::Vector3d res;
    res = e1_ + (RY_*RX_*curUavState_.GetVel())/controlGain_.c;
    
    return res;
}

const Eigen::Vector3d
HomographyGeometric::UpdateError2FromEstVel()
{
    Eigen::Vector3d res;
    res = e1_ + (velVitualEst_)/controlGain_.c;
    
    return res;
}


const Eigen::Vector3d 
HomographyGeometric::UpdateFVitual()
{
    Eigen::Vector3d res;
    res = - controlGain_.Kv * e2_;
    return res;
}

const double 
HomographyGeometric::UpdateThrust()
{
    double res;
    res = - (FVitual_ - curUavState_.GetMass() * curUavState_.GetGravity() * RZ_*axisZ_).transpose()*(RY_*RX_*axisZ_);
    // Common::ShowVal("res",res);
    res = (res - curUavState_.GetMass() * curUavState_.GetGravity())*thrustScale_ + thrustOffest_;
    
    return res;
}

const Eigen::Matrix3d 
HomographyGeometric::UpdateRotationDesired()
{
    Eigen::Matrix3d res;
    Eigen::Vector3d b1d,b2d,b3d;
    Eigen::Vector3d dot_b1d,dot_b2d,dot_b3d;
    Eigen::Vector3d dot_b1c;
    Eigen::Vector3d tmp,dot_tmp;//tmp代表临时变量 局部变量

    dot_yawDesired_ =(yawDesired_ - yawDesiredLast_)*controlRate_;
    dot_b1c << -sin(yawDesired_)*dot_yawDesired_,cos(yawDesired_)*dot_yawDesired_,0;
    yawDesiredLast_ = yawDesired_;
    b3d = RZ_ * (- FVitual_/curUavState_.GetMass() + curUavState_.GetGravity()*axisZ_ );
    tmp = b3d;
    //限幅处理
    //
    b3d.normalize();//归一化
    b2d = b3d.cross(b1c_);//叉乘
    b2d.normalize();
    b1d = b2d.cross(b3d);
    b1d.normalize();

    // Eigen::Vector3d debugVal1,debugVal2,debugVal3;//调试变量  仅调试用
//     debugVal1 = RZ_*Common::MatrixHat(curUavState_.GetOmega()(2)*axisZ_)*(- FVitual_/curUavState_.GetMass() + curUavState_.GetGravity()*axisZ_ );
// debugVal2 = RZ_*((controlGain_.Kv/(curUavState_.GetMass()*controlGain_.c) * 
// ( -Common::MatrixHat(curUavState_.GetOmega()(2)*axisZ_)*e1_ + (  RY_*RX_*curUavState_.GetAcc())/controlGain_.c + controlGain_.c *(e2_-e1_))));
    // dot_tmp = debugVal1 + debugVal2;
                 
    dot_tmp = RZ_*Common::MatrixHat(curUavState_.GetOmega()(2)*axisZ_)*(- FVitual_/curUavState_.GetMass() + curUavState_.GetGravity()*axisZ_ )
                + RZ_*((controlGain_.Kv/(curUavState_.GetMass()*controlGain_.c) * 
                 ( -Common::MatrixHat(curUavState_.GetOmega()(2)*axisZ_)*e1_ + (  RY_*RX_*curUavState_.GetAcc())/controlGain_.c + controlGain_.c *(e2_-e1_))));

    dot_b3d = b3d.cross(dot_tmp/tmp.norm()).cross(b3d);
    dot_b2d = b2d.cross((dot_b3d.cross(b1c_) + dot_b1c.cross(b3d))/b3d.cross(b1c_).norm()).cross(b2d);
    dot_b1d = dot_b2d.cross(b3d) + dot_b3d.cross(b2d);

    res = Common::VectorToMatrix(b1d,b2d,b3d);
    dot_RDesired_ = Common::VectorToMatrix(dot_b1d,dot_b2d,dot_b3d);

    // Common::ShowVal("debugVal1",debugVal1);
    // Common::ShowVal("debugVal2",debugVal2);

    return res;
}


const Eigen::Vector3d 
HomographyGeometric::UpdateOmegaDesired()
{
    Eigen::Vector3d res;
    res = Common::MatrixHatInv(RDesired_.transpose()*dot_RDesired_);
    return res;
}

const Eigen::Vector3d 
HomographyGeometric::UpdateVelocityEstimation()
{
    
    // static Eigen::Vector3d  e1ErrorTmp = Eigen::Vector3d::Zero();
    // static Eigen::Vector3d  e1EstTmp = Eigen::Vector3d::Zero();
    // static Eigen::Vector3d  velVitualErrorTmp = Eigen::Vector3d::Zero();
    // static Eigen::Vector3d  velVitualEstTmp = Eigen::Vector3d::Zero();
    double aStar_ = 1.0;

    Eigen::Vector3d dot_e1EatTmp = - Common::MatrixHat(curUavState_.GetOmega()(2) * axisZ_)*e1Error_
                                    + aStar_*velVitualEst_ + controlGain_.k1*e1Error_;
    e1Est_ = e1Est_ + dot_e1EatTmp/controlRate_;
    e1Error_ = e1_ - e1Est_;

    // Eigen::Vector3d dot_velVitualEstTmp = -Common::MatrixHat(curUavState_.GetOmega()(2) * axisZ_)*velVitualEst_
    //                                     + RY_*RX_*curUavState_.GetAcc() + controlGain_.k2*e1Error_;
    Eigen::Vector3d dot_velVitualEstTmp = -Common::MatrixHat(curUavState_.GetOmega()(2) * axisZ_)*velVitualEst_
                                        + RY_*RX_*curUavState_.GetAcc() + controlGain_.k2*e1Error_ + controlGain_.k3*velVitualError_;
    velVitualEst_ = velVitualEst_ + dot_velVitualEstTmp/controlRate_;
    // velVitualError_ = RY_*RX_*curUavState_.GetVel() - velVitualEst_;
    velVitualError_ = velVitualEstToTrue_ - velVitualEst_;
    velVitualEstToTrue_ = velVitualEst_;
    return velVitualEst_;
}

const Eigen::Vector3d& 
HomographyGeometric::GetOmegaDesired()const
{
    return omegaDesired_;
}


void 
HomographyGeometric::operator() (const Control::Quadrotor& curUavState)
{

    Eigen::Matrix3d enu_R_ned;
    enu_R_ned << 1.0, 0.0, 0.0,
                 0.0, -1.0, 0.0, 
                 0.0, 0.0, -1.0;
    SetCurUavState(curUavState);//外部传的state为相对与世界坐标系 
    //两次转换 
    curUavState_.Vector3dFrameTransform(curUavState_.GetR().transpose());
    curUavState_.Vector3dFrameTransform(enu_R_ned.transpose());
    //相对坐标系转换
    curUavState_.RotationFrameTransform(enu_R_ned.transpose(),enu_R_ned);
    //角速度单独转换 因为默认都是 base_x_base，现在需要 baseNed_x_naseNed
    Eigen::Vector3d omega(curUavState_.GetOmega());
    omega = enu_R_ned.transpose() * omega;
    curUavState_.SetOmega(omega);

    //更新内部变量
    // Eigen::Vector3d angle(curUavState_.GetEulerAngle());
    Eigen::Vector3d angle(curUavState_.GetEulerAngle());
    

    angle -= eulerAngleOffest_;

    RZ_ << cos(angle(0)),-sin(angle(0)),0,sin(angle(0)),cos(angle(0)),0,0,0,1;
    RY_ << cos(angle(1)),0,sin(angle(1)),0,1,0,-sin(angle(1)),0,cos(angle(1));
    RX_ << 1,0,0,0,cos(angle(2)),-sin(angle(2)),0,sin(angle(2)),cos(angle(2));
    
    yawFromHomographyVirtual_ = UpdateyawFromHomographyVirtual();//arcsin(0.5 * vec(hv_T-hv)_T * ez)
    RZFromHomographyVirtual_ << cos(yawFromHomographyVirtual_),-sin(yawFromHomographyVirtual_),0,sin(yawFromHomographyVirtual_),cos(yawFromHomographyVirtual_),0,0,0,1;
    //更新当前的旋转矩阵
    curUavState_.SetR(RZFromHomographyVirtual_*RY_*RX_);

    e1_ = UpdateError1();//e1 = (I-Hv)*m

    e2_ = UpdateError2FromTrueVel();//e2 = e1 + Vv/c
    e2_ = UpdateError2FromEstVel();
    FVitual_ = UpdateFVitual();//Fv = -Kv * e2

    thrust_ = UpdateThrust();//T = - [Fv - m*g*ez]' (RY*RX *ez);
    RDesired_ = UpdateRotationDesired();//FVitual_ -> RDesired_
    omegaDesired_ = UpdateOmegaDesired();
    velVitualEst_ = UpdateVelocityEstimation();//估计虚拟框架当前速度
    // Common::ShowVal("RZ_",RZ_);
    // Common::ShowVal("RY_",RY_);
    // Common::ShowVal("RX_",RX_);
    // Common::ShowVal("angle(0)",angle(0));

    // Common::ShowVal("yawFromHomographyVirtual_",yawFromHomographyVirtual_);
    curUavState_.ShowState("homo",5);
    ShowInternal(5);
    // ShowParamVal(5);
}

const Control::Quadrotor& 
HomographyGeometric::GetQuadrotor()const
{
    return curUavState_;
}

const Eigen::Matrix3d& 
HomographyGeometric::GetRDesired()const
{
    return RDesired_;
}

const double& 
HomographyGeometric::GetThrust()const
{
    return thrust_;
}

const double& 
HomographyGeometric::GetControlRate() const
{
    return controlRate_;
}

const Eigen::Vector3d&
HomographyGeometric::GetE1() const
{
    return e1_;
}

const Eigen::Vector3d&
HomographyGeometric::GetE2() const
{
    return e2_;
}

const Eigen::Vector3d&
HomographyGeometric::GetE1Error() const
{
    return e1Error_;
}

const Eigen::Vector3d&
HomographyGeometric::GetE1Est() const
{
    return e1Est_;
}

const Eigen::Vector3d&
HomographyGeometric::GetVelVitualError() const
{
    return velVitualError_;
}

const Eigen::Vector3d&
HomographyGeometric::GetVelVitualEst() const
{
    return velVitualEst_;
}

const Eigen::Vector3d 
HomographyGeometric::GetVelVitualTure() const
{
    return RY_*RX_*curUavState_.GetVel();
}

void 
HomographyGeometric::ShowInternal(int num) const
{
    std::cout << "-------------------HomographyFGeometric-------------------" <<std::endl;
    Common::ShowVal("homography_",homography_,num);
    Common::ShowVal("homographyVirtual_",homographyVirtual_,num);
    Common::ShowVal("e1_",e1_,num);
    Common::ShowVal("e2_",e2_,num);
    Common::ShowVal("FVitual_",FVitual_,num);
    Common::ShowVal("thrust_",thrust_,num);
    Common::ShowVal("RDesired_",RDesired_,num);
    Common::ShowVal("dot_RDesired_",dot_RDesired_,num);
    Common::ShowVal("omegaDesired_",omegaDesired_,num);
    Common::ShowVal("velVitualEst_",velVitualEst_);
    Common::ShowVal("RY_*RX_*curUavState_.GetVel()",RY_*RX_*curUavState_.GetVel());
    Common::ShowVal("velVitualError_",velVitualError_);

}

void 
HomographyGeometric::ShowParamVal(int num) const
{
    Common::ShowVal("yawDesired_",yawDesired_,num);
    Common::ShowVal("controlGain_.c",controlGain_.c,num);
    Common::ShowVal("controlGain_.Kv",controlGain_.Kv,num);
    Common::ShowVal("thrustOffest_",thrustOffest_,num);
    Common::ShowVal("thrustScale_",thrustScale_,num);
}
}