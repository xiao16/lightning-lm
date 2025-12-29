//
// Created by xiang on 23-12-14.
//

#include "bag_io.h"

#include <glog/logging.h>
#include <boost/foreach.hpp>
#include <filesystem>

namespace lightning {

void RosbagIO::Go(int sleep_usec) {
    std::filesystem::path p(bag_file_);
    
    try {
        rosbag::Bag bag;
        bag.open(bag_file_, rosbag::bagmode::Read);

        // 收集所有要订阅的topic
        std::vector<std::string> topics;
        for (const auto& kv : cloud2_funcs_) {
            topics.push_back(kv.first);
        }
        for (const auto& kv : livox_funcs_) {
            topics.push_back(kv.first);
        }
        for (const auto& kv : imu_funcs_) {
            topics.push_back(kv.first);
        }

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for (const rosbag::MessageInstance& m : view) {
            // 处理 PointCloud2
            if (cloud2_funcs_.count(m.getTopic())) {
                sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
                if (cloud != nullptr) {
                    cloud2_funcs_[m.getTopic()](cloud);
                }
            }

            // 处理 Livox
            if (livox_funcs_.count(m.getTopic())) {
                livox_ros_driver2::msg::CustomMsg::ConstPtr cloud =
                    m.instantiate<livox_ros_driver2::msg::CustomMsg>();
                if (cloud != nullptr) {
                    livox_funcs_[m.getTopic()](cloud);
                }
            }

            // 处理 IMU
            if (imu_funcs_.count(m.getTopic())) {
                sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                if (imu_msg != nullptr) {
                    IMUPtr imu = std::make_shared<IMU>();
                    imu->timestamp = ToSec(imu_msg->header.stamp);

                    /// NOTE: 如果需要乘重力，请修改此处
                    imu->linear_acceleration = Vec3d(imu_msg->linear_acceleration.x, 
                                                     imu_msg->linear_acceleration.y,
                                                     imu_msg->linear_acceleration.z);
                    imu->angular_velocity = Vec3d(imu_msg->angular_velocity.x, 
                                                  imu_msg->angular_velocity.y,
                                                  imu_msg->angular_velocity.z);

                    imu_funcs_[m.getTopic()](imu);
                }
            }

            if (sleep_usec > 0) {
                usleep(sleep_usec);
            }

            if (lightning::debug::flg_exit) {
                bag.close();
                return;
            }
        }

        bag.close();
        LOG(INFO) << "bag " << bag_file_ << " finished.";
    } catch (rosbag::BagException& e) {
        LOG(ERROR) << "Error opening bag file: " << e.what();
    }
}

}  // namespace lightning
