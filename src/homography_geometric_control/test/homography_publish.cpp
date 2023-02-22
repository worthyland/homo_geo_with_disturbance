#include <mavros_interaction/MavrosInteraction.h>
#include <homo_msgs/HomographyResult.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp> 



ros::Time totalBegin,totalEnd;


class HomographyPublish
{
private:
    /* data */
    ros::NodeHandle nh_,nhParam_;
    ros::Subscriber imageSub_;
    ros::Publisher resultsPub_;
    ros::Publisher  imagePub_;
    int pubChoose_;// 是否发布 0不发布 1
    std::string  refImgPath_;//参考图片路径
    std::string refImgName_;//参考图片名称
    bool foundRef_,foundCur_;//是否检测到点的标志位
    std::vector<cv::Point2f> cornersRef_, cornersCur_;//角点存储ref cur
    int witdh_,hight_;//图片的大小
    cv::Mat HMat_;//单应性矩阵 cv::Mat格式 从图像处计算
    Eigen::Matrix3d HMatrix_;//单应性矩阵 Eigen::Matrix3d格式 从图像处计算
    Eigen::Matrix3d cameraK_;//相机内参

    cv::Mat curImg_,refImg_;//图片存储

    homo_msgs::HomographyResult homoMatrixFromPose_,homoMatrixFromImg_;
    //从位置计算单应性矩阵所需参数
    Eigen::Vector3d posDesire_;
    Eigen::Vector3d n_;
    double d_;
    Eigen::Vector3d refned_posError_basened_;
    Eigen::Matrix3d refned_homo_basened_;//单应性矩阵 Eigen::Matrix3d格式 从位置处计算

    void ImageCb(const sensor_msgs::ImageConstPtr& msg);
    cv::Scalar randomColor(int64 seed);
    homo_msgs::HomographyResult HomographyCalculationFromPose(const Control::Quadrotor::state& curState);
public:
    HomographyPublish(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~HomographyPublish();

    void operator() (const Control::Quadrotor::state& curState);

};



HomographyPublish::HomographyPublish(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam):nh_(nh),nhParam_(nhParam)
{
    nhParam_.param("witdh",witdh_,640);
    nhParam_.param("hight",hight_,480);
        //初始化posDesire
    nhParam_.param("posDesire/x",posDesire_(0),0.0);
    nhParam_.param("posDesire/y",posDesire_(1),0.0);
    nhParam_.param("posDesire/z",posDesire_(2),1.0);
    //初始化n
    nhParam_.param("n/x",n_(0),0.0);
    nhParam_.param("n/y",n_(1),0.0);
    nhParam_.param("n/z",n_(2),1.0);
    //初始化d
    nhParam_.param("d",d_,1.0);

    nhParam.param("pubChoose",pubChoose_,0);
    nhParam.param<std::string>("refImgPath",refImgPath_,"/home/sw/homo_geo/src/homography_geometric_control/ref_img/");
    nhParam.param<std::string>("refImgName",refImgName_,"image_1.png");

    if(pubChoose_ == 2 ||pubChoose_ == 4){
        imageSub_ = nh_.subscribe("/image_raw",10,&HomographyPublish::ImageCb,this);// /iris/usb_cam    /galaxy_camera
    }
    imagePub_ = nh_.advertise<sensor_msgs::Image>("/image_draw",5,true);
    resultsPub_ = nh_.advertise<homo_msgs::HomographyResult>("homography_pose", 1,true);

    HMatrix_ = Eigen::Matrix3d::Identity();
    cameraK_ <<554.3827128, 0.0, 320.5, 0.0, 554.3827128, 240.5, 0.0, 0.0, 1.0;
    refImg_ = cv::imread(refImgPath_+refImgName_,cv::IMREAD_GRAYSCALE);
    double witdhRate = witdh_/(double)refImg_.cols;
    double hightRate = hight_/(double)refImg_.rows;
    cv::resize(refImg_,refImg_,cv::Size(0,0),witdhRate,hightRate);
    foundRef_ = cv::findChessboardCorners(refImg_, cv::Size (4,3), cornersRef_);

}

HomographyPublish::~HomographyPublish()
{
}

void
HomographyPublish::ImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cvPtr;
    try
    {
        cvPtr = cv_bridge::toCvCopy(msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    curImg_ = cvPtr->image; 

    double witdhRate = witdh_/(double)curImg_.cols;
    double hightRate = hight_/(double)curImg_.rows;
    // std::cout << "witdh_rate:"<<witdh_rate<<"\nhight_rate:"<<hight_rate<<std::endl;
    cv::resize(curImg_,curImg_,cv::Size(0,0),witdhRate,hightRate);
    // std::cout << "img_witdh:"<<curImg.cols<<"\nimhg_hight:"<<curImg.rows<<std::endl;

    foundCur_ = cv::findChessboardCorners(curImg_, cv::Size (4,3), cornersCur_);   
    // std::cout << "foundCur:"<<foundCur<<std::endl;
    if(foundCur_ == true){
        HMat_ = cv::findHomography(cornersRef_, cornersCur_);
        // std::cout << "HMat_:\n" << HMat_ << std::endl;
        cv::cv2eigen(HMat_,HMatrix_);
        // HMatrix_(0,0) = HMat_.at<uchar>(0,0);HMatrix_(0,1) = HMat_.at<uchar>(0,1);HMatrix_(0,2) = HMat_.at<uchar>(0,2);
        // HMatrix_(1,0) = HMat_.at<uchar>(1,0);HMatrix_(1,1) = HMat_.at<uchar>(1,1);HMatrix_(1,2) = HMat_.at<uchar>(1,2);
        // HMatrix_(2,0) = HMat_.at<uchar>(2,0);HMatrix_(2,1) = HMat_.at<uchar>(2,1);HMatrix_(2,2) = HMat_.at<uchar>(2,2);  
        // std::cout << "HMatrix_:\n" << HMatrix_ << std::endl;
        HMatrix_ = cameraK_.inverse()* HMatrix_ * cameraK_;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(HMatrix_,Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd C(3,1);
        C = svd.singularValues();
        // std::cout << "C:\n" << C << std::endl;
        HMatrix_ = HMatrix_/C(1,0);    
    // std::cout << "HMatrix_2:\n" << HMatrix_ << std::endl;
        cv::Mat img_draw_matches,ref_img_rgb8,cur_img_rgb8;
        cv::cvtColor(refImg_,ref_img_rgb8,cv::COLOR_GRAY2RGB);
        cv::cvtColor(curImg_,cur_img_rgb8,cv::COLOR_GRAY2RGB);
        cv::hconcat(ref_img_rgb8, cur_img_rgb8, img_draw_matches);
        for (size_t i = 0; i < cornersRef_.size(); i++)
        {
            cv::Mat pt1 = (cv::Mat_<double>(3,1) << cornersRef_[i].x, cornersRef_[i].y, 1);
            cv::Mat pt2 = HMat_ * pt1;
            pt2 /= pt2.at<double>(2);
            cv::Point end( (int) (refImg_.cols + pt2.at<double>(0)), (int) pt2.at<double>(1) );
            line(img_draw_matches, cornersRef_[i], end, randomColor(cv::getTickCount()), 1);
        }    
        cvPtr->encoding = sensor_msgs::image_encodings::RGB8;
        cvPtr->image = img_draw_matches;
        imagePub_.publish(cvPtr->toImageMsg());  
    }else{
        HMatrix_ = Eigen::Matrix3d::Identity();
    }
    
    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
                homoMatrixFromImg_.homography[i*3 + j] =HMatrix_(i,j);
                
        }  
    }

}

homo_msgs::HomographyResult 
HomographyPublish::HomographyCalculationFromPose(const Control::Quadrotor::state& curState)
{
    Eigen::Matrix3d enu_R_ned;
    enu_R_ned << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
    Eigen::Vector3d ref_pos_base(curState.pos);//

    Eigen::Vector3d ref_posError_base(ref_pos_base - posDesire_);
    refned_posError_basened_ = enu_R_ned*ref_posError_base;
    

    Eigen::Matrix3d ref_R_base = Eigen::Matrix3d::Identity();
    ref_R_base = curState.orientation.toRotationMatrix();
    // cout << "R:"<<enu_R_ned <<endl;
    Eigen::Matrix3d refned_R_basened = Eigen::Matrix3d::Identity();;
    refned_R_basened = enu_R_ned.transpose()*ref_R_base*enu_R_ned;
    // cout << "R2:"<<enu_R_ned <<endl;
     


    refned_homo_basened_ = refned_R_basened.transpose() - 
                (refned_R_basened.transpose() * refned_posError_basened_ * n_.transpose()) / d_;


    homo_msgs::HomographyResult res;

    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
                res.homography[i*3 + j] =refned_homo_basened_(i,j);
                
        }  
    }
    return res;
}

cv::Scalar 
HomographyPublish::randomColor(int64 seed)
{
    cv::RNG rng(seed);
    int icolor = (unsigned)rng;
    return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}


void 
HomographyPublish::operator() (const Control::Quadrotor::state& curState)
{
   homoMatrixFromPose_ =HomographyCalculationFromPose(curState);
    if(pubChoose_ == 0){}
    else if(pubChoose_ == 1 || pubChoose_ == 4){
        resultsPub_.publish(homoMatrixFromPose_);
        std::cout << "!!!!!!!!!!!!!!!!!!!!HomographyPubFromPose!!!!!!!!!!!!!!!!!!!!"<< std::endl;
    }else if(pubChoose_ == 2){
        resultsPub_.publish(homoMatrixFromImg_);
        std::cout << "!!!!!!!!!!!!!!!!!!!!HomographyPubFromImg!!!!!!!!!!!!!!!!!!!!"<< std::endl;
    }

    std::cout << "--------------HomographyCalculationFromPose----------------"<< std::endl;
    Common::ShowVal("refned_posError_basened_",refned_posError_basened_,5);
    Common::ShowVal("refned_homo_basened_",refned_homo_basened_,5);

    std::cout << "--------------HomographyCalculationFromImg----------------"<< std::endl;
    Common::ShowVal("refImgPath+refImgName",refImgPath_+refImgName_,5);
    Common::ShowVal("foundRef_",foundRef_,5);
    Common::ShowVal("foundCur_",foundCur_,5);
    Common::ShowVal("HMatrix_",HMatrix_,5);

}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "homography_publish_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhParam("~");//加“～”才能读取

    
    HomographyPublish homographyPublish(nh,nhParam);

    Control::MavrosInteraction uavInfo(nh,nhParam);

    
    ros::Rate rate(60.0);
    while(ros::ok()){
        
        homographyPublish(uavInfo.GetState());
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}