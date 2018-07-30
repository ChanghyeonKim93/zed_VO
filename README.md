This is a visual odometry (VO) navigation ROS node for PIONEER-3DX based on ZED positional tracking.


##Environment
NVIDIA TX1 (ARM) / JetPack 3.0 (Ubuntu 16.04 LTS) / ZED SDK 2.0 / ROS Kinetic


##Algorithm
6 DoF camera mostion is estimated with ZED positional tracking functions (VO + VSLAM).


##How to compile?
Provided with this repo is a CMakeLists.txt file, which you can use to directly compile the code as follows:
```bash
$ cd ~/catkin_ws/src/
$ git clone https://pjinkim@bitbucket.org/icslmoon/icslrover_vo.git
$ cd ~/catkin_ws/
$ catkin_make
```


##Source command
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ source devel/setup.sh
```


##Contact
For any queries, contact: 


##License
MIT
