# Fusing IMU and Odometry

This is a demo fusing IMU data and Odometry data (wheel odom or Lidar odom) or GPS data to obtain better odometry.

Currently, I implement Extended Kalman Filter (EKF), batch optimization and isam2 to fuse IMU and Odometry data.

## Dependencies

This demo has the following dependencies:

* [ROS](http://www.ros.org/)
  * I test it on ROS melodic and kinetic version

* [GTSAM](https://bitbucket.org/gtborg/gtsam/src/develop/)
  * use tag 4.28a now

* [Kalman Filter Library](https://github.com/mherb/kalman)

## Build

Clone the repository and catkin_make:

``` bash
cd ~/catkin_ws/src
git clone https://github.com/Guo-ziwei/fusion
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

run EKF:

``` bash
rosrun ekf_fusion ekf_fusion_node bag_filename.bag
```

run batch:

``` bash
rosrun gtsam_imu imu_preintegrated bag_filename.bag
```

it will take a long time to get the result and I will fix it in the future

run isam2:

``` bash
rosrun gtsam_imu imu_preintegrated_sm bag_filename.bag
```

## Performance

Test on [MIt dataset](https://projects.csail.mit.edu/stata/index.php)

<img src="https://github.com/Guo-ziwei/fusion/blob/master/image/result_ekf.png" width = 50% height = 50% div align=center />

## TODO

Now the demo is running off-line (reading data from bag) and I will turn it to online.
