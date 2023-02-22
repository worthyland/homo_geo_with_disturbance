#ifndef __COMMON__
#define __COMMON__

/**
 * @brief Common library 
 * 
 */
#include <Eigen/Eigen>
#include <cmath>

namespace Common
{


static Eigen::Matrix3d 
VectorToMatrix(const Eigen::Vector3d& v1,const Eigen::Vector3d& v2,const Eigen::Vector3d& v3)
{   
    Eigen::Matrix3d res;
    res <<  v1(0),v2(0),v3(0),
            v1(1),v2(1),v3(1),
            v1(2),v2(2),v3(2);
    return res;
} 

static Eigen::Vector3d 
MatrixHatInv(const Eigen::Matrix3d& val)
{
    Eigen::Vector3d res;
    res << 0.5 * (val(2,1) - val(1,2)), 0.5 * (val(0,2) - val(2,0)), 0.5 * (val(1,0) - val(0,1));
    return res;
}

static Eigen::Matrix3d 
MatrixHat(const Eigen::Vector3d& val)
{
    Eigen::Matrix3d res;
    res << 0, -val(2), val(1), 
            val(2), 0, -val(0), 
            -val(1), val(0), 0;
    return res;
}

static Eigen::Matrix3d 
LogMatrix(const Eigen::Matrix3d& val)
{
    Eigen::Matrix3d res;
    double tmp;
    tmp = acos((val.trace()-1)/2);
    res = tmp/(2*sin(tmp))*(val-val.transpose());
    return res;
}

static Eigen::Vector3d 
Limit(const Eigen::Vector3d& val)
{
    Eigen::Vector3d res;
    
    return res;
}

static Eigen::Vector3d 
Sign(const Eigen::Vector3d& val)
{
    Eigen::Vector3d res;
    for(int i=0;i<3;i++){
        if(val(i) > 0) res(i) = 1;
        else if(val(i) == 0) res(i) = 0;
        else res(i) = -1;
    }
    return res;
}

static void 
ShowVal(const std::string& str,const Eigen::Vector3d& val,int num=5) 
{
    std::cout << str <<":";
    std::cout<<std::fixed<< std::setprecision(num)<< val(0)<<","<<val(1)<<","<<val(2)<<","<<std::endl;
}

static void 
ShowVal(const std::string& str,const Eigen::Matrix3d& val,int num=5) 
{
    std::cout << str << ":" << std::endl;
    std::cout<<std::fixed<< std::setprecision(num)<< val<<std::endl;
}


static void 
ShowVal(const std::string& str,const Eigen::Quaterniond& val,int num=5) 
{
    std::cout << str <<":";
    std::cout<<std::fixed<< std::setprecision(num)<< val.w()<<","<<val.x()<<","
                                                <<val.y()<<","<<val.z()<<std::endl;
}
template<typename T>
static void 
ShowVal(const std::string& str,const T& val,int num=5)
{
    std::cout << str <<":";
    std::cout<<std::fixed<< std::setprecision(num)<< val <<std::endl;
}




} // namespace name




#endif