//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros1 node ... ";

    /// subscribers
    nh_ = std::make_shared<ros::NodeHandle>();
    pnh_ = std::make_shared<ros::NodeHandle>("~");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    imu_sub_ = nh_->subscribe<sensor_msgs::Imu>(
        imu_topic_, 100, [this](const sensor_msgs::Imu::ConstPtr& msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = nh_->subscribe<sensor_msgs::PointCloud2>(
        cloud_topic_, 100, [this](const sensor_msgs::PointCloud2::ConstPtr& cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = nh_->subscribe<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, 100, [this](const livox_ros_driver2::msg::CustomMsg::ConstPtr& cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
        loc_->SetTFCallback(
            [this](const geometry_msgs::TransformStamped &pose) { 
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pose.transform.translation.x, 
                                                 pose.transform.translation.y, 
                                                 pose.transform.translation.z));
                tf::Quaternion q(pose.transform.rotation.x, 
                                 pose.transform.rotation.y, 
                                 pose.transform.rotation.z, 
                                 pose.transform.rotation.w);
                transform.setRotation(q);
                
                tf_broadcaster_->sendTransform(
                    tf::StampedTransform(transform, 
                                        ros::Time(pose.header.stamp.sec, pose.header.stamp.nanosec),
                                        pose.header.frame_id, 
                                        pose.child_frame_id));
            });
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::ConstPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (nh_ != nullptr) {
        ros::spin();
    }
}

}  // namespace lightning