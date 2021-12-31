# Livox-Localization

This repository implements a point-cloud map based localization framework. The odometry information is published with  [FAST-LIO](https://github.com/hku-mars/FAST_LIO). And the  initial localization information is obtained with [ScanContext](https://github.com/irapkaist/scancontext) and its python-implementation  [PyICP-SLAM](https://github.com/gisbi-kim/PyICP-SLAM). The main idea is inspired by the [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION). This repository is the extension of our mapping module: [Livox-Mapping](https://github.com/PJLab-ADG/Livox-Mapping), where you can get an accurate point-cloud map.

## 1. Features
- Realtime 3D global localization in a pre-built point cloud map. 
- The initial localization pose can be obtained with ScanContext which store the history point cloud collection information.

![demo](images/Livox-Localization.gif)

## 2. Prerequisites
### 2.1 Dependencies for FAST-LIO

Technically, if you have built and run FAST-LIO before, you may skip section 2.1.

This part of dependency is consistent with FAST-LIO, please refer to the documentation [[Link]( https://github.com/hku-mars/FAST_LIO#1-prerequisites)].

Also, we have provided you the Docker Image (For Cpp-Dependencies) for a quick usage, please check [DockerHub](https://hub.docker.com/repository/docker/siyuanhuang95/livox_slam).  For specific information, please refer to the document described in [Livox-Mapping](https://github.com/PJLab-ADG/Livox-Mapping#23-docker).

### 2.2 Dependencies for localization module

- python 2.7

- [ros_numpy](https://github.com/eric-wieser/ros_numpy)

```shell
sudo apt install ros-$ROS_DISTRO-ros-numpy
```

- [Open3D](http://www.open3d.org/docs/0.9.0/getting_started.html)

```shell
pip install open3d
```

- numpy

```shell
pip install numpy
```

- sklearn

```shell
pip install sklearn
```



## 3. Build

Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com//SiyuanHuang95/Livox-Localization.git
    cd Livox-Localization
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow [livox_ros_driver](https://github.com/hku-mars/FAST_LIO#13-livox_ros_driver))




## 4. Run Localization
### 4.1 Dataset Requirements

 Before running the mapping functionality, please make sure the sensor data are published to the correct rostopic.

- Livox LiDAR: /livox/lidar
- IMU: /livox/imu

### 4.2 Sample Dataset

For the ease of usage, we are providing several test rosbags collected in one industrial park located in Shanghai. Please be aware that all codes and datasets included in this repository are for academic research purposes only. Other usages are **NOT** encouraged, and it is at your own risk. If You have any concerns including privacy, please contact us by sending an e-mail to [huangsiyuan@pjlab.org.cn](mailto:huangsiyuan@pjlab.org.cn)

The dataset can be downloaded through the Baidu Netdisk with:

```
Link：https://pan.baidu.com/s/17ElBOWiFVr68975FtXY8ZA 
Passwort：pjop
```

The pre-built point-cloud map and its corresponding  extracted history information represented with Ring Key, Scan Context and the pose file could be found in one zip file saved in the Baidu Netdisk.

### 4.3 Prepare ScanContext information

Before run the localization module, you have to prepare the ScanContext related files, which store the history odometry information during the collection activity. Here, we assume that the odometry information has been store in the format of  [**interactive_slam**](https://github.com/SMRT-AIST/interactive_slam). That format is also the output representation of our former work  [Livox-Mapping](https://github.com/PJLab-ADG/Livox-Mapping). 

We provide the function *livox_load_pc_make_sc* in our *ScanContextManager* class, you can refer to the file *livox_scan_context_test.py* for the usage demonstration.

### 4.4 Run Localization Module

1. First, please make sure you're using the **Python 2.7** environment;
1. Provide ScanContext information path. Modify the SC-file path for line 256 of the file global_localization.
1. Run localization, here we take Livox Horizion as an example:

```shell
 roslaunch livox_localization localization_horizon.launch  map:=/path/to/your/map.pcd
```

​	Please modify `/path/to/your/map.pcd` to your own map point cloud file path.

​	Wait for 3~5 seconds until the map cloud shows up in RVIZ;

4. Play the demo rosbag.

```shell
rosbag play demo.bag
```


## Related Works
1. [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry (LIO) package
2. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
3. [FAST-LIO-SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): The integration of FAST-LIO with [Scan-Context](https://github.com/irapkaist/scancontext) **loop closure** module.
3. [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): A simple localization framework that can re-localize in built maps based on FAST-LIO.
4. [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization): A simple system that can relocalize a robot on a built map based on LIO-SAM.


## Acknowledgments
Thanks for the authors of [FAST-LIO](https://github.com/hku-mars/FAST_LIO),  [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) and [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization). We also use the tools from  [PyICP-SLAM](https://github.com/gisbi-kim/PyICP-SLAM) and [kd-tree](https://github.com/stefankoegl/kdtree).

## Contact

- If you have any questions, contact here please
  - Send email to 
    - Siyuan Huang: [GitHub]( https://github.com/SiyuanHuang95) or  [Email]( huangsiyuan@pjlab.org.cn )
    - Xudong Zhao: [GitHub]( https://github.com/zxd123) or [Email]( zhaoxudong@pjlab.org.cn ):
  - Report issues on GitHub.