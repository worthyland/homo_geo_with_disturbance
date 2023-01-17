#include <homography_geometric_control/HomographyGeometricControl.h>
#include <mavros_interaction/MavrosInteraction.h>
#include <so3_control/SO3Control.h>
#include <std_msgs/Float32MultiArray.h>

using namespace Control;
using namespace  std;
using namespace  HomographyGeometricControl;


int main(int argc, char *argv[]){

    ros::init(argc, argv, "homo_geo_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhParam("~");

    MavrosInteraction uavInfo(nh,nhParam);//与mavros交互，获取无人机状态 ref_X_base
    HomographyGeometric outLoop(nh,nhParam);
    SO3Control attitudeControllor(nh,nhParam);

    ros::Publisher recordPub = nh.advertise<std_msgs::Float32MultiArray>
        ("record",10);


    ros::Rate rate(outLoop.GetControlRate());
    std::cout<<"wait for the connected!"<<std::endl;

    while(ros::ok() && !uavInfo.GetCurrentControlState().connected){
        std::cout<<"connecting.....: "<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }


    while(ros::ok()){
        
        // uavInfo.ShowUavState(5);//通过mavros读取的
        outLoop(uavInfo.GetQuadrotor());
        attitudeControllor(outLoop.GetRDesired(),outLoop.GetOmegaDesired(),outLoop.GetQuadrotor());
        Eigen::Matrix3d enu_R_ned;
        enu_R_ned << 1.0, 0.0, 0.0,0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
        //力矩计算表示在base_link_ned，推力计算为正，不需要转换
        uavInfo.ActuatorPub(outLoop.GetThrust(), attitudeControllor.GetTorque(),true);
        // uavInfo.ActuatorPub();

        std_msgs::Float32MultiArray outputRecord;

        uavInfo.DataSvae(outputRecord,outLoop.GetE1());//0 1 2
        uavInfo.DataSvae(outputRecord,outLoop.GetE2());//3 4 5
        uavInfo.DataSvae(outputRecord,outLoop.GetE1Error());//6 7 8 
        uavInfo.DataSvae(outputRecord,outLoop.GetE1Est());//9 10 11
        uavInfo.DataSvae(outputRecord,outLoop.GetVelVitualError());//12 13 14
        uavInfo.DataSvae(outputRecord,outLoop.GetVelVitualEst());//15 16 17
        uavInfo.DataSvae(outputRecord,outLoop.GetVelVitualTure());// 18 19 20

        uavInfo.DataSvae(outputRecord,outLoop.GetQuadrotor().GetEulerAngle() *180 /M_PI);//21 22 23

        uavInfo.DataSvae(outputRecord,attitudeControllor.GetTorque()); // 24 25 26
        uavInfo.DataSvae(outputRecord,attitudeControllor.GetER());//27 28 29
        uavInfo.DataSvae(outputRecord,attitudeControllor.GetEOmega());// 30 31 32
        uavInfo.DataSvae(outputRecord,outLoop.GetThrust());// 33
        uavInfo.DataSvae(outputRecord,attitudeControllor.GetAttitudeTrackError());// 34
        recordPub.publish(outputRecord);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}