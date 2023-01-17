# 说明
此代码用以调试基于单应性的几何控制，考虑扰动的情况下

# 定义环境变量
source devel/setup.sh

函数命名采用大驼峰法

普通变量命名采用小驼峰法

与数学相关的变量命名为：
ref_posDesire_base（机体坐标系在参考坐标系的期望位置  "_"将之分成三部分：1.参考的坐标系 2.变量名称含义 3.当前的坐标系）





修改mavros中IMU的发布频率为250hz

rosrun mavros mavcmd long 511 31 4000 0 0 0 0 0

511 为修改指令

31 为imu的编码 , 对应的话题名称为mavros/imu/data (103 对应的话题名称为mavros/imu/data_raw)

4000 为4000us，对应为250hz

后面五个零固定写法

rosrun mavros mavcmd long 511 32 16666 0 0 0 0 0 
//（mavros/local_position/pose）60hz


rosbag record /record /mavros/local_position/pose /image_draw

# 安装mavros

sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras

cd /opt/ros/noetic/lib/mavros //or (wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh)

sudo ./install_geographiclib_datasets.sh

# 常用软件
VScode：sudo dpkg -i ***.deb

Terminator：sudo apt install terminator

Plotjuggler：

sudo apt install ros-noetic-plotjuggler

sudo apt install ros-noetic-plotjuggler-ros

rosrun plotjuggler plotjuggler

Net-tools：

sudo apt install net-tools

ifconfig

ssh：

sudo apt install openssh-server

ssh 192.168.**.**(ssh 远程电脑ipv4地址（同一局域网）)

远程桌面

推荐 nomachine


# 真机实验事项
## ssh远程连接
ssh linux@192.168.2.149
## 启动无人机
sudo chmod 777 /dev/ttyUSB0  

roslaunch mavros px4.launch gcs_url:=udp://@192.168.2.100
## 运行动作捕捉系统
roslaunch vrpn_client_ros sample.launch server:=192.168.2.135
## 将动捕的位置发送给无人机
rosrun topic_tools relay /vrpn_client_node/uav1/pose /mavros/vision_pose/pose 


# 查看数据是否正常
rostopic echo /mavros/vision_pose/pose

rostopic echo /mavros/local_position/pose

roslaunch ima-lanind position_test.launch arg =  (qifei)


## sw_test
roslaunch galaxy_camera MER-139.launch 

rosrun sw_uav_homo sw_e3_control_rpy_with_dynamics

rosrun sw_uav_homo sw_homo_Chessboard

cd /media/robot/G

#记录需要的话题
  /mavros/imu/data /mavros/local_position/velocity_body /tracking /sw/image_raw /mavros/local_position/pose

rosbag record /output_record //（自定义的话题名称，提供内部状态变量  数据类型为数组）
### 如果桌面vscode打不开 执行如下：
sudo chmod 755 /

# 录像  输出默认保存为output.avi
rosrun image_view video_recorder image:=/galaxy_camera/image_raw
# 记录摄像头话题
rosbag record /galaxy_camera/image_raw

rosbag play ()   /galaxy_camera/image_raw:=/camera/image_raw
