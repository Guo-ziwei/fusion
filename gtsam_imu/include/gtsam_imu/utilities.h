#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <map>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <string>
#include <vector>

void LoadImufromBag(std::string bag_filename, std::vector<sensor_msgs::Imu>& imu_data) {
    rosbag::Bag bag;
    bag.open(bag_filename);
    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
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
    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
        geometry_msgs::PoseWithCovarianceStampedPtr wheel = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (wheel != nullptr) {
            wheel_odom.push_back(*wheel);
        }
    }
    bag.close();
}

enum SensorType { RTK, IMU };

struct ImuData {
    ImuData() {
        timestamp = 0;
        ax = 0.0;
        ay = 0.0;
        az = 0.0;
        gx = 0.0;
        gy = 0.0;
        gz = 0.0;
    }
    ImuData(double _timestamp, double _ax, double _ay, double _az, double _gx, double _gy, double _gz)
        : timestamp(_timestamp), ax(_ax), ay(_ay), az(_az), gx(_gx), gy(_gy), gz(_gz){};
    double timestamp, ax, ay, az, gx, gy, gz;
};

struct RTKData {
    RTKData() {
        timestamp = 0;
        x = 0.0;
        y = 0.0;
        z = 0.0;
        cov_x = 0.0;
        cov_y = 0.0;
        cov_z = 0.0;
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
        cov_vx = 0.0;
        cov_vy = 0.0;
        cov_vz = 0.0;
        state = 0;
    }
    RTKData(
        double _timestamp, double _x, double _y, double _z, double _cov_x, double _cov_y, double _cov_z, double _vx,
        double _vy, double _vz, double _cov_vx, double _cov_vy, double _cov_vz, int _state)
        : timestamp(_timestamp),
          x(_x),
          y(_y),
          z(_z),
          cov_x(_cov_x),
          cov_y(_cov_y),
          cov_z(_cov_z),
          vx(_vx),
          vy(_vy),
          vz(_vz),
          cov_vx(_cov_vx),
          cov_vy(_cov_vy),
          cov_vz(_cov_vz),
          state(_state) {}
    double timestamp, x, y, z, cov_x, cov_y, cov_z, vx, vy, vz, cov_vx, cov_vy, cov_vz;
    int state;
};

bool loadIMU(std::string datafile, std::vector<ImuData>& imudata) {
    FILE* file;
    file = fopen(datafile.c_str(), "r");
    if (file == nullptr) {
        std::cerr << "cannot find IMU file" << std::endl;
        return false;
    }
    double ax, ay, az, gx, gy, gz;
    uint64_t timestamp_ns;
    while (fscanf(file, "%ld %lf %lf %lf %lf %lf %lf", &timestamp_ns, &ax, &ay, &az, &gx, &gy, &gz) != EOF) {
        const auto&& imu = ImuData(static_cast<double>(timestamp_ns) * 1e-9, ax, ay, az, gx, gy, gz);
        imudata.push_back(imu);
    }
    fclose(file);
    std::cout << "load imu data done" << std::endl;
    return true;
}

bool loadRTK(std::string datafile, std::vector<RTKData>& rtkdata) {
    FILE* file;
    file = fopen(datafile.c_str(), "r");
    if (file == nullptr) {
        std::cerr << "cannot find RTK file" << std::endl;
        return false;
    }
    int state, delay;
    double time_stamp, x, y, z, cov_x, cov_y, cov_z, vx, vy, vz, cov_vx, cov_vy, cov_vz, rtcmlost;
    while (fscanf(
               file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d", &time_stamp, &x, &y, &z, &cov_x,
               &cov_y, &cov_z, &vx, &vy, &vz, &cov_vx, &cov_vy, &cov_vz, &rtcmlost, &state, &delay) != EOF) {
        const auto&& rtk = RTKData(time_stamp, x, y, z, cov_x, cov_y, cov_z, vx, vy, vz, cov_vx, cov_vy, cov_vz, state);
        rtkdata.push_back(rtk);
    }
    fclose(file);
    std::cout << "load RTK data done" << std::endl;
    return true;
}

bool loadAllSensorTime(
    std::multimap<double, std::pair<uint64_t, SensorType>>& allsensortime, const std::vector<ImuData>& imudata,
    const std::vector<RTKData>& gpsdata) {
    unsigned long index{0};
    for (const auto& imu : imudata) {
        std::pair<uint64_t, SensorType> sensorpair(index, SensorType::IMU);
        allsensortime.emplace(std::make_pair(imu.timestamp, sensorpair));
        index++;
    }
    index = 0;
    for (const auto& gps : gpsdata) {
        std::pair<uint64_t, SensorType> sensorpair(index, SensorType::RTK);
        allsensortime.emplace(std::make_pair(gps.timestamp, sensorpair));
        index++;
    }
    // std::sort(allsensortime.begin(), allsensortime.end(), mapKeyCmp);
    std::cout << "sort data done" << std::endl;
    return true;
}

#endif