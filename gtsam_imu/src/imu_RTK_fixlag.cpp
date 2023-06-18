#include <boost/shared_ptr.hpp>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>
// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

#include "gtsam_imu/utilities.h"

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

// IMU preintegration parameters.
boost::shared_ptr<PreintegratedImuMeasurements::Params> imuParams() {
    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = 0.2000;
    double gyro_noise_sigma = 0.0150;
    double accel_bias_rw_sigma = 0.045;
    double gyro_bias_rw_sigma = 0.0001;
    Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
    Matrix33 integration_error_cov =
        I_3x3 * 1e-6;  // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
    Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
    Matrix66 bias_acc_omega_init = I_6x6 * 1e-5;  // error in the bias used for preintegration

    auto p = PreintegratedImuMeasurements::Params::MakeSharedU(9.8);
    // PreintegrationBase params:
    p->accelerometerCovariance = measured_acc_cov;     // acc white noise in continuous
    p->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov;  // gyro white noise in continuous
    // PreintegrationCombinedMeasurements params:
    // p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
    // p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
    // p->biasAccOmegaInt = bias_acc_omega_init;

    return p;
}

int main(int argc, char const* argv[]) {
    string imufile{
        "/run/user/1000/gvfs/sftp:host=10.10.42.90/home/guozw/testdata/dm-vio/DVT-0026/"
        "inter_imu.txt"},
        rtkfile{
            "/run/user/1000/gvfs/sftp:host=10.10.42.90/home/guozw/testdata/dm-vio/DVT-0026/"
            "gps.txt"},
        output_file;
    vector<ImuData> imudata;
    vector<RTKData> rtkdata;
    multimap<double, std::pair<uint64_t, SensorType>> allsensortime;
    loadIMU(imufile, imudata);
    loadRTK(rtkfile, rtkdata);
    loadAllSensorTime(allsensortime, imudata, rtkdata);
    // Define the smoother lag (in seconds)
    double lag = 2.0;
    BatchFixedLagSmoother smootherBatch(lag);
    FixedLagSmoother::KeyTimestampMap newTimestamps;
    Values initial_values;
    Rot3 prior_rotation = Rot3::Quaternion(0.932341, 0.056385, -0.0620969, -0.351716);
    Vector3 prior_point(-0.877, 3.180, -0.732);
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(0.0, 0.0, 0.0);
    Point3 p_G_I(0.312358, -0.02574, -0.05691);
    imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias
    int correction_count = 0;              // The number of corrections we have received
    initial_values.insert(X(correction_count), prior_pose);
    initial_values.insert(V(correction_count), prior_velocity);
    initial_values.insert(B(correction_count), prior_imu_bias);

    auto pose_noise_model =
        noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.5))
                                         .finished());                 // rad,rad,rad,m, m, m
    auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
    auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior factors (pose, velocity, bias) to the graph.
    NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    graph->addPrior(X(correction_count), prior_pose, pose_noise_model);
    graph->addPrior(V(correction_count), prior_velocity, velocity_noise_model);
    graph->addPrior(B(correction_count), prior_imu_bias, bias_noise_model);

    auto p = imuParams();
    auto preintegrated = boost::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);

    // Store previous state for imu integration and latest predicted outcome.
    NavState prev_state(prior_pose, prior_velocity);
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = prior_imu_bias;

    // Keep track of total error over the entire run as simple performance metric.
    double current_position_error = 0.0, current_orientation_error = 0.0,
           init_timestamp = static_cast<double>(1683798517406413568) * 1e-9;
    newTimestamps[X(correction_count)] = init_timestamp;
    newTimestamps[V(correction_count)] = init_timestamp;
    newTimestamps[B(correction_count)] = init_timestamp;
    auto sensor_index = allsensortime.find(init_timestamp);
    for (; sensor_index != allsensortime.end(); sensor_index++) {
        switch (sensor_index->second.second) {
            case SensorType::IMU: {
                if (sensor_index->second.first == 0) {
                    break;
                }
                double dt = imudata[sensor_index->second.first].timestamp -
                            imudata[sensor_index->second.first - 1].timestamp;
                preintegrated->integrateMeasurement(
                    Vector3(
                        imudata[sensor_index->second.first].ax,
                        imudata[sensor_index->second.first].ay,
                        imudata[sensor_index->second.first].az),
                    Vector3(
                        imudata[sensor_index->second.first].gx,
                        imudata[sensor_index->second.first].gy,
                        imudata[sensor_index->second.first].gz),
                    dt);
                break;
            }
            case SensorType::RTK: {
                /* code */
                correction_count++;
                newTimestamps[X(correction_count)] = sensor_index->first;
                newTimestamps[V(correction_count)] = sensor_index->first;
                newTimestamps[B(correction_count)] = sensor_index->first;
                // Adding IMU factor and RTK position factor and optimizing.
                auto preint_imu = dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
                prop_state = preintegrated->predict(prev_state, prev_bias);
                ImuFactor imu_factor(
                    X(correction_count - 1), V(correction_count - 1), X(correction_count),
                    V(correction_count), B(correction_count - 1), preint_imu);
                graph->add(imu_factor);
                Point3 rtk_measurement =
                    prop_state.attitude() * p_G_I + Point3(
                                                        rtkdata[sensor_index->second.first].x,
                                                        rtkdata[sensor_index->second.first].y,
                                                        rtkdata[sensor_index->second.first].z);
                imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
                graph->add(BetweenFactor<imuBias::ConstantBias>(
                    B(correction_count - 1), B(correction_count), zero_bias, bias_noise_model));
                GPSFactor gps_factor(
                    X(correction_count), rtk_measurement,
                    noiseModel::Diagonal::Sigmas(Vector3(
                        rtkdata[sensor_index->second.first].cov_x,
                        rtkdata[sensor_index->second.first].cov_y,
                        rtkdata[sensor_index->second.first].cov_z)));
                graph->add(gps_factor);

                // Now optimize and compare results.
                initial_values.insert(X(correction_count), prop_state.pose());
                initial_values.insert(V(correction_count), prop_state.v());
                initial_values.insert(B(correction_count), prev_bias);
                // for (const auto& key_timestamp : newTimestamps) {
                //     cout << " Key to add: " << key_timestamp.first
                //          << "  Time: " << to_string(key_timestamp.second) << endl;
                // }
                smootherBatch.update(*graph, initial_values, newTimestamps);
                // for (const auto& key_timestamp : smootherBatch.timestamps()) {
                //     cout << " Key: " << key_timestamp.first
                //          << "  Time: " << to_string(key_timestamp.second) << endl;
                // }
                Values result = smootherBatch.calculateEstimate();
                Pose3 current_pose = result.at<Pose3>(X(correction_count));
                Vector3 current_velocity = result.at<Vector3>(V(correction_count));
                prev_state = NavState(
                    result.at<Pose3>(X(correction_count)), result.at<Vector3>(V(correction_count)));
                prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));
                // Reset the preintegration object.
                preintegrated->resetIntegrationAndSetBias(prev_bias);
                // Print the optimized current pose
                std::cout << "Timestamp = " << to_string(sensor_index->first) << " ";
                std::cout << current_pose.translation().transpose() << " ";
                std::cout << current_pose.rotation().toQuaternion().coeffs().transpose() << " ";
                std::cout << current_velocity.transpose() << std::endl;
                // Clear contains for the next iteration
                newTimestamps.clear();
                initial_values.clear();
                graph->resize(0);
                // if (initial_values.exists(B(correction_count))) {
                //     std::cout << "B "<<(correction_count)<<" exists" << std::endl;
                // }
                // initial_values.insert(X(correction_count), current_pose);
                // initial_values.insert(V(correction_count), current_velocity);
                // initial_values.insert(B(correction_count), prev_bias);
                // graph->addPrior(X(correction_count), current_pose, pose_noise_model);
                // graph->addPrior(V(correction_count), current_velocity, velocity_noise_model);
                // graph->addPrior(B(correction_count), prev_bias, bias_noise_model);
                // newTimestamps[correction_count] = sensor_index->first;
                break;
            }
            default:
                std::cerr << "unknown sensor type" << std::endl;
                break;
        }
    }

    return 0;
}
