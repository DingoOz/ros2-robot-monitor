#include "ros2_robot_monitor/topic_monitor.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <thread>

TopicMonitor::TopicMonitor() : Node("topic_monitor") {
    // Open debug file
    debug_file_.open("/tmp/ros2_monitor_debug.log", std::ios::app);
    debug_file_ << "=== Monitor started ===" << std::endl;
    debug_file_.flush();
    
    // Set up a timer to periodically update topic activity and discover new topics
    // Use a shorter interval to catch topics that appear after startup
    auto timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            debug_file_ << "Timer callback triggered" << std::endl;
            debug_file_.flush();
            update_topic_activity();
            discover_topics();
        });
    
    debug_file_ << "Timer created, starting initial discovery" << std::endl;
    debug_file_.flush();
    
    // Wait a bit for the ROS2 graph to stabilize, then do initial discovery
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    discover_topics();
}

void TopicMonitor::discover_topics() {
    debug_file_ << "discover_topics() called" << std::endl;
    debug_file_.flush();
    
    auto topic_names_and_types = this->get_topic_names_and_types();
    
    debug_file_ << "Discovering topics, found " << topic_names_and_types.size() << " topics" << std::endl;
    debug_file_.flush();
    
    for (const auto& [topic_name, topic_types] : topic_names_and_types) {
        debug_file_ << "Found topic: " << topic_name << std::endl;
        debug_file_.flush();
        
        // Skip internal ROS topics
        if (topic_name.find("/rosout") != std::string::npos ||
            topic_name.find("/parameter_events") != std::string::npos) {
            debug_file_ << "Skipping internal topic: " << topic_name << std::endl;
            continue;
        }
        
        // Skip if already subscribed
        if (subscribers_.find(topic_name) != subscribers_.end()) {
            debug_file_ << "Already subscribed to: " << topic_name << std::endl;
            continue;
        }
        
        // Find supported message type
        for (const auto& topic_type : topic_types) {
            debug_file_ << "Topic " << topic_name << " has type: " << topic_type << std::endl;
            std::lock_guard<std::mutex> lock(data_mutex_);
            topic_data_[topic_name].type = topic_type;
            
            if (topic_type == "std_msgs/msg/String") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<std_msgs::msg::String>(
                    topic_name, qos,
                    [this, topic_name](const std_msgs::msg::String::SharedPtr msg) {
                        string_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "std_msgs/msg/Int32") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<std_msgs::msg::Int32>(
                    topic_name, qos,
                    [this, topic_name](const std_msgs::msg::Int32::SharedPtr msg) {
                        int32_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "std_msgs/msg/Float32") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<std_msgs::msg::Float32>(
                    topic_name, qos,
                    [this, topic_name](const std_msgs::msg::Float32::SharedPtr msg) {
                        float32_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "std_msgs/msg/Bool") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<std_msgs::msg::Bool>(
                    topic_name, qos,
                    [this, topic_name](const std_msgs::msg::Bool::SharedPtr msg) {
                        bool_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "geometry_msgs/msg/Twist") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<geometry_msgs::msg::Twist>(
                    topic_name, qos,
                    [this, topic_name](const geometry_msgs::msg::Twist::SharedPtr msg) {
                        twist_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "nav_msgs/msg/Odometry") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<nav_msgs::msg::Odometry>(
                    topic_name, qos,
                    [this, topic_name](const nav_msgs::msg::Odometry::SharedPtr msg) {
                        odometry_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "sensor_msgs/msg/LaserScan") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    topic_name, qos,
                    [this, topic_name](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                        laser_scan_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "sensor_msgs/msg/Imu") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<sensor_msgs::msg::Imu>(
                    topic_name, qos,
                    [this, topic_name](const sensor_msgs::msg::Imu::SharedPtr msg) {
                        imu_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                debug_file_ << "Successfully subscribed to " << topic_name << " as IMU" << std::endl;
                debug_file_.flush();
                break;
            } else if (topic_type == "sensor_msgs/msg/BatteryState") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<sensor_msgs::msg::BatteryState>(
                    topic_name, qos,
                    [this, topic_name](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                        battery_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "sensor_msgs/msg/Temperature") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<sensor_msgs::msg::Temperature>(
                    topic_name, qos,
                    [this, topic_name](const sensor_msgs::msg::Temperature::SharedPtr msg) {
                        temperature_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "sensor_msgs/msg/Image") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<sensor_msgs::msg::Image>(
                    topic_name, qos,
                    [this, topic_name](const sensor_msgs::msg::Image::SharedPtr msg) {
                        image_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            } else if (topic_type == "diagnostic_msgs/msg/DiagnosticArray") {
                auto qos = rclcpp::QoS(1).best_effort();
                auto subscription = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                    topic_name, qos,
                    [this, topic_name](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
                        diagnostic_callback(topic_name, msg);
                    });
                subscribers_[topic_name] = subscription;
                break;
            }
        }
    }
}


void TopicMonitor::update_topic_frequency(const std::string& topic_name) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto& info = topic_data_[topic_name];
    auto now = std::chrono::steady_clock::now();
    
    info.msg_count++;
    info.last_time = now;
    info.msg_history.push_back(now);
    
    // Keep only recent messages for frequency calculation
    auto cutoff = now - std::chrono::seconds(5);
    while (!info.msg_history.empty() && info.msg_history.front() < cutoff) {
        info.msg_history.pop_front();
    }
    
    // Calculate frequency
    if (info.msg_history.size() >= 2) {
        auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(
            info.msg_history.back() - info.msg_history.front()).count();
        if (time_span > 0) {
            info.frequency = static_cast<double>(info.msg_history.size() - 1) * 1000.0 / time_span;
        }
    }
}

void TopicMonitor::update_topic_activity() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto now = std::chrono::steady_clock::now();
    
    for (auto& [topic_name, info] : topic_data_) {
        auto time_since_last = std::chrono::duration_cast<std::chrono::seconds>(
            now - info.last_time).count();
        info.is_active = (time_since_last < 5);
    }
}

std::map<std::string, TopicInfo> TopicMonitor::get_topic_data() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return topic_data_;
}

std::deque<ImuData> TopicMonitor::get_imu_data(const std::string& topic_name) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = imu_data_.find(topic_name);
    if (it != imu_data_.end()) {
        return it->second;
    }
    return std::deque<ImuData>();
}

std::vector<std::string> TopicMonitor::get_imu_topics() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<std::string> imu_topics;
    for (const auto& [topic_name, topic_info] : topic_data_) {
        if (topic_info.type == "sensor_msgs/msg/Imu") {
            imu_topics.push_back(topic_name);
        }
    }
    return imu_topics;
}

// Message callback implementations
void TopicMonitor::string_callback(const std::string& topic_name, const std_msgs::msg::String::SharedPtr msg) {
    topic_data_[topic_name].last_data = "'" + msg->data + "'";
    update_topic_frequency(topic_name);
}

void TopicMonitor::int32_callback(const std::string& topic_name, const std_msgs::msg::Int32::SharedPtr msg) {
    topic_data_[topic_name].last_data = std::to_string(msg->data);
    update_topic_frequency(topic_name);
}

void TopicMonitor::float32_callback(const std::string& topic_name, const std_msgs::msg::Float32::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << msg->data;
    topic_data_[topic_name].last_data = oss.str();
    update_topic_frequency(topic_name);
}

void TopicMonitor::bool_callback(const std::string& topic_name, const std_msgs::msg::Bool::SharedPtr msg) {
    topic_data_[topic_name].last_data = msg->data ? "true" : "false";
    update_topic_frequency(topic_name);
}

void TopicMonitor::twist_callback(const std::string& topic_name, const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "lin:[" << msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "] "
        << "ang:[" << msg->angular.x << "," << msg->angular.y << "," << msg->angular.z << "]";
    topic_data_[topic_name].last_data = oss.str();
    update_topic_frequency(topic_name);
}

void TopicMonitor::odometry_callback(const std::string& topic_name, const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "pos:[" << msg->pose.pose.position.x << "," 
        << msg->pose.pose.position.y << "," << msg->pose.pose.position.z << "]";
    topic_data_[topic_name].last_data = oss.str();
    update_topic_frequency(topic_name);
}

void TopicMonitor::laser_scan_callback(const std::string& topic_name, const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!msg->ranges.empty()) {
        auto min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        auto max_range = *std::max_element(msg->ranges.begin(), msg->ranges.end());
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "ranges: " << msg->ranges.size() << " points, min: " << min_range 
            << "m, max: " << max_range << "m";
        topic_data_[topic_name].last_data = oss.str();
    } else {
        topic_data_[topic_name].last_data = "Empty scan";
    }
    update_topic_frequency(topic_name);
}

void TopicMonitor::imu_callback(const std::string& topic_name, const sensor_msgs::msg::Imu::SharedPtr msg) {
    debug_file_ << "Received IMU message on " << topic_name << std::endl;
    
    // Store IMU data for charting (with separate lock scope)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        ImuData imu_data;
        imu_data.timestamp = std::chrono::steady_clock::now();
        imu_data.acc_x = msg->linear_acceleration.x;
        imu_data.acc_y = msg->linear_acceleration.y;
        imu_data.acc_z = msg->linear_acceleration.z;
        imu_data.gyro_x = msg->angular_velocity.x;
        imu_data.gyro_y = msg->angular_velocity.y;
        imu_data.gyro_z = msg->angular_velocity.z;
        
        imu_data_[topic_name].push_back(imu_data);
        
        // Keep only last 10 seconds of data (assuming ~100Hz IMU)
        auto cutoff = imu_data.timestamp - std::chrono::seconds(10);
        while (!imu_data_[topic_name].empty() && imu_data_[topic_name].front().timestamp < cutoff) {
            imu_data_[topic_name].pop_front();
        }
        
        // Update display string
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "acc:[" << msg->linear_acceleration.x << "," 
            << msg->linear_acceleration.y << "," << msg->linear_acceleration.z << "]";
        topic_data_[topic_name].last_data = oss.str();
    }
    
    // Update frequency outside the lock to avoid deadlock
    update_topic_frequency(topic_name);
}

void TopicMonitor::battery_callback(const std::string& topic_name, const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "voltage: " << msg->voltage << "V, percentage: " << msg->percentage << "%";
    topic_data_[topic_name].last_data = oss.str();
    update_topic_frequency(topic_name);
}

void TopicMonitor::temperature_callback(const std::string& topic_name, const sensor_msgs::msg::Temperature::SharedPtr msg) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << msg->temperature << "°C";
    topic_data_[topic_name].last_data = oss.str();
    update_topic_frequency(topic_name);
}

void TopicMonitor::image_callback(const std::string& topic_name, const sensor_msgs::msg::Image::SharedPtr msg) {
    std::ostringstream oss;
    oss << msg->width << "x" << msg->height << " " << msg->encoding;
    topic_data_[topic_name].last_data = oss.str();
    update_topic_frequency(topic_name);
}

void TopicMonitor::diagnostic_callback(const std::string& topic_name, const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    std::ostringstream oss;
    oss << msg->status.size() << " diagnostics";
    if (!msg->status.empty()) {
        oss << ": " << msg->status[0].name;
        if (msg->status.size() > 1) {
            oss << ", " << msg->status[1].name;
        }
        if (msg->status.size() > 2) {
            oss << ", ...";
        }
    }
    topic_data_[topic_name].last_data = oss.str();
    update_topic_frequency(topic_name);
}