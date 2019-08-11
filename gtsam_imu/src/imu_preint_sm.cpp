#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam_imu/utilities.h>

using namespace gtsam;

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

PreintegrationType *imu_preintegrated_;

int main(int argc, char const *argv[])
{
  std::string bag_name;
  std::vector<sensor_msgs::Imu> imu_data;
  std::vector<geometry_msgs::PoseWithCovarianceStamped> wheel_odom;
  if (argc < 2) {
      std::cout<<"please give the rosbag file path!"<<std::endl;
      return 1;
  } else {
      bag_name = argv[1];
  }
  LoadImufromBag(bag_name, imu_data);
  LoadWheelOdomfromBag(bag_name, wheel_odom);
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  ISAM2 isam(parameters);
  Rot3 prior_rotation = Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);
  Point3 prior_position(0.0, 0.0, 0.0);
  Pose3 prior_pose(prior_rotation, prior_position);
  Vector3 prior_vel(0.0, 0.0, 0.0);
  imuBias::ConstantBias prior_imu_bias;
  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_vel);
  initial_values.insert(B(correction_count), prior_imu_bias);

  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas(
                                                        (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());// rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose3>>(X(correction_count), prior_pose, pose_noise_model);
  graph.emplace_shared<PriorFactor<Vector3>>(V(correction_count), prior_vel,velocity_noise_model);
  graph.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(correction_count), prior_imu_bias,bias_noise_model);

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0980;
  double gyro_noise_sigma = 0.0013;
  // double accel_bias_rw_sigma = 0.055;
  // double gyro_bias_rw_sigma = 0.00015;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
  // Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  // Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  // Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

  boost::shared_ptr<PreintegratedImuMeasurements::Params> p = PreintegratedImuMeasurements::Params::MakeSharedU();
  // PreintegrationBase params:
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuoustgf
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  p->setUse2ndOrderCoriolis(true);
  // // PreintegrationCombinedMeasurements params:
  // p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  // p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  // p->biasAccOmegaInt = bias_acc_omega_int;

  imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);
  double dt = 0.01;
  NavState prev_state(prior_pose, prior_vel);
  NavState last_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // All priors have been set up, now iterate through the data.
  Eigen::Matrix<double, 6, 1> imu = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 7, 1> odom = Eigen::Matrix<double, 7, 1>::Zero();
  for (auto iter_odom = wheel_odom.begin()+1; iter_odom != wheel_odom.end(); ++iter_odom) {
    auto del_point = imu_data.begin();
    for (auto iter_imu = imu_data.begin(); iter_imu->header.stamp <= iter_odom->header.stamp; ++iter_imu) {
      imu<<-iter_imu->linear_acceleration.x, iter_imu->linear_acceleration.y, -iter_imu->linear_acceleration.z,
            -iter_imu->angular_velocity.x, iter_imu->angular_velocity.y, -iter_imu->angular_velocity.z;
      imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
      del_point = iter_imu;
    }
    odom<<iter_odom->pose.pose.position.x, iter_odom->pose.pose.position.y, iter_odom->pose.pose.position.z,
            iter_odom->pose.pose.orientation.w, iter_odom->pose.pose.orientation.x, 
            iter_odom->pose.pose.orientation.y, iter_odom->pose.pose.orientation.z;
    correction_count++;
    PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
    ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                          X(correction_count), V(correction_count),
                          B(correction_count-1), *preint_imu);
    graph.add(imu_factor);
    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph.add(BetweenFactor<imuBias::ConstantBias>( B(correction_count-1),
                                                    B(correction_count  ),
                                                    zero_bias, bias_noise_model ));
    noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3, 1, 0);
    GPSFactor gps_factor(X(correction_count), Point3(odom(0), odom(1), odom(2)), correction_noise);
    graph.add(gps_factor);
    // Now optimize and compare results.
    last_state = imu_preintegrated_->predict(prev_state, prev_bias);
    initial_values.insert(X(correction_count), last_state.pose());
    initial_values.insert(V(correction_count), last_state.v());
    initial_values.insert(B(correction_count), prev_bias);
    isam.update(graph, initial_values);
    isam.update();
    Values result = isam.calculateEstimate();
    // Overwrite the beginning of the preintegration for the next step.
    prev_state = NavState(result.at<Pose3>(X(correction_count)),
                          result.at<Vector3>(V(correction_count)));
    prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));
    // Reset the preintegration object.
    imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
    graph.resize(0);
    initial_values.clear();
    // Print out the position and orientation.
    Vector3 gtsam_position = prev_state.pose().translation();
    Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
    std::cout<<0.0<<" "<<0.0<<" "<<gtsam_position(0)<<" "<<gtsam_position(1)<<" "<<gtsam_position(2)<<" "
              <<gtsam_quat.x()<<" "<<gtsam_quat.y()<<" "<<gtsam_quat.z()<<" "<<gtsam_quat.w()<<std::endl;
    imu_data.erase(imu_data.begin(), del_point);
  }
  return 0;
}