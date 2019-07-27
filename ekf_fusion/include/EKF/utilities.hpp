#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <vector>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>


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

void LoadWheelOdomfromBag(std::string bag_filename, std::vector<geometry_msgs::PoseWithCovarianceStamped>& measure) {
  rosbag::Bag bag;
  bag.open(bag_filename);
  // double time;
  for (rosbag::MessageInstance const m:rosbag::View(bag)) {
      geometry_msgs::PoseWithCovarianceStampedConstPtr wheel_odom = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
      if (wheel_odom != nullptr) {
        measure.push_back(*wheel_odom);
        // data.timestamp = imu->header.stamp.toSec();
      }
  }
  bag.close();
}
#endif