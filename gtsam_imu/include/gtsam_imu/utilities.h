#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <string>

void LoadImufromBag(std::string bag_filename, std::vector<sensor_msgs::Imu>& imu_data) {
  rosbag::Bag bag;
  bag.open(bag_filename);
  for (rosbag::MessageInstance const m:rosbag::View(bag)) {
      sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
      if (imu != nullptr) {
        imu_data.push_back(*imu);
      }
  }
  bag.close();
}

void LoadWheelOdomfromBag(std::string bag_filename, std::vector<geometry_msgs::PoseWithCovarianceStamped>& wheel_odom) {
  rosbag::Bag bag;
  bag.open(bag_filename);
  for (rosbag::MessageInstance const m:rosbag::View(bag)) {
      geometry_msgs::PoseWithCovarianceStampedPtr wheel = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
      if (wheel != nullptr) {
        wheel_odom.push_back(*wheel);
      }
  }
  bag.close();
}


#endif