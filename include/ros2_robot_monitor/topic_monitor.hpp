#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <string>
#include <map>
#include <memory>
#include <deque>
#include <chrono>
#include <mutex>
#include <fstream>

struct ImuData {
    std::chrono::steady_clock::time_point timestamp;
    double acc_x, acc_y, acc_z;
    double gyro_x, gyro_y, gyro_z;
};

struct TopicInfo {
    std::string type;
    double frequency;
    std::string last_data;
    int msg_count;
    std::chrono::steady_clock::time_point last_time;
    std::deque<std::chrono::steady_clock::time_point> msg_history;
    bool is_active;
    
    TopicInfo() : frequency(0.0), msg_count(0), is_active(false) {
        msg_history.clear();
    }
};

class TopicMonitor : public rclcpp::Node {
public:
    TopicMonitor();
    
    void discover_topics();
    std::map<std::string, TopicInfo> get_topic_data() const;
    std::deque<ImuData> get_imu_data(const std::string& topic_name) const;
    std::vector<std::string> get_imu_topics() const;
    
private:
    std::map<std::string, TopicInfo> topic_data_;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    std::map<std::string, std::deque<ImuData>> imu_data_;
    mutable std::mutex data_mutex_;
    std::ofstream debug_file_;
    
    void update_topic_frequency(const std::string& topic_name);
    void update_topic_activity();
    
    
    // Message callback handlers
    void string_callback(const std::string& topic_name, const std_msgs::msg::String::SharedPtr msg);
    void int32_callback(const std::string& topic_name, const std_msgs::msg::Int32::SharedPtr msg);
    void float32_callback(const std::string& topic_name, const std_msgs::msg::Float32::SharedPtr msg);
    void bool_callback(const std::string& topic_name, const std_msgs::msg::Bool::SharedPtr msg);
    void twist_callback(const std::string& topic_name, const geometry_msgs::msg::Twist::SharedPtr msg);
    void odometry_callback(const std::string& topic_name, const nav_msgs::msg::Odometry::SharedPtr msg);
    void laser_scan_callback(const std::string& topic_name, const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void imu_callback(const std::string& topic_name, const sensor_msgs::msg::Imu::SharedPtr msg);
    void battery_callback(const std::string& topic_name, const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void temperature_callback(const std::string& topic_name, const sensor_msgs::msg::Temperature::SharedPtr msg);
    void image_callback(const std::string& topic_name, const sensor_msgs::msg::Image::SharedPtr msg);
    void diagnostic_callback(const std::string& topic_name, const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
};