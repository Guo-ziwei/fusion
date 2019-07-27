#define _USE_MATH_DEFINES
#include <cmath>

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>

#include "EKF/systemmodel.hpp"
#include "EKF/PoseMeasurementModel.hpp"
#include "EKF/utilities.hpp"

typedef float T;

// Some type shortcuts
typedef Planar_Robot::State<T> State;
typedef Planar_Robot::Control<T> Control;
typedef Planar_Robot::SystemModel<T> SystemModel;
typedef Planar_Robot::PoseMeasurement<T> PoseMeasure;
typedef Planar_Robot::PoseMeasurementModel<T> PoseMeasureModel;


int main(int argc, char const **argv)
{
  std::string bag_filename = "/home/guoziwei/Documents/2011-01-25-06-29-26.bag";
  if (argc < 2) {
    std::cout<<"please give the rosbag file path!"<<std::endl;
    return 1;
  } else {
    bag_filename = argv[1];
  }
  State x_ekf;
  x_ekf.setZero();
  x_ekf.qw() = 1.0;

  Control u;

  SystemModel sys;

  PoseMeasureModel pose_measurement;

  Kalman::ExtendedKalmanFilter<State> predictor;
  Kalman::ExtendedKalmanFilter<State> ekf;

  predictor.init(x_ekf);
  ekf.init(x_ekf);

  std::vector<sensor_msgs::Imu> imu_data;
  std::vector<geometry_msgs::PoseWithCovarianceStamped> wheel_odom;

  LoadImufromBag(bag_filename, imu_data);

  LoadWheelOdomfromBag(bag_filename, wheel_odom);

  State x_pred;
  for (auto iter_odom = wheel_odom.begin()+1; iter_odom != wheel_odom.end(); ++iter_odom) {
    PoseMeasure measure;
    auto del_position = imu_data.begin();
    for (auto iter_imu = imu_data.begin(); iter_imu->header.stamp <= iter_odom->header.stamp; ++iter_imu) {
      u.a_x() = -iter_imu->linear_acceleration.x;
      u.a_y() = iter_imu->linear_acceleration.y;
      u.a_z() = 0.0;
      u.w_x() = 0.0;
      u.w_y() = 0.0;
      u.w_z() = -iter_imu->angular_velocity.z;
      // x_ekf = sys.f(x_ekf, u);
      // x_pred = predictor.predict(sys, u);
      x_ekf = ekf.predict(sys, u);
      del_position = iter_imu;
    }
    measure.x() = iter_odom->pose.pose.position.x;
    measure.y() = iter_odom->pose.pose.position.y;
    measure.z() = 0.0;
    measure.qw() = iter_odom->pose.pose.orientation.w;
    measure.qx() = iter_odom->pose.pose.orientation.x;
    measure.qy() = iter_odom->pose.pose.orientation.y;
    measure.qz() = iter_odom->pose.pose.orientation.z;
    x_ekf = ekf.update(pose_measurement, measure);
    imu_data.erase(imu_data.cbegin(), del_position);
    
    // Print to stdout as csv format
    // std::cout   << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta() << ","
    //             << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta()  << ","
    //             << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta()  << ","
    //             << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta()
    //             << std::endl;
    // Print to stdout as plot format
    std::cout<<0.0<<" "<<0.0<<" "<<x_ekf.x()<<" "<<x_ekf.y()<<" "<<x_ekf.z()<<" "<<x_ekf.qx()<<" "<<x_ekf.qy()<<" "<<x_ekf.qz()<<" "<<x_ekf.qw()<<std::endl;
    // std::cout<<0.0<<" "<<0.0<<" "<<x_pred.x()<<" "<<x_pred.y()<<" "<<x_pred.z()<<" "<<x_pred.qx()<<" "<<x_pred.qy()<<" "<<x_pred.qz()<<" "<<x_pred.qw()<<std::endl;
  }


  return 0;
}
