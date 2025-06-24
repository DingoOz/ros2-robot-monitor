#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <thread>
#include <chrono>
#include <iostream>

#include "ros2_robot_monitor/topic_monitor.hpp"
#include "ros2_robot_monitor/ncurses_display.hpp"

std::shared_ptr<TopicMonitor> g_monitor;
std::unique_ptr<NCursesDisplay> g_display;
bool g_running = true;

void signal_handler(int signum) {
    g_running = false;
    if (g_display) {
        g_display->cleanup_display();
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv) {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    try {
        // Initialize ROS2
        rclcpp::init(argc, argv);
        
        // Create monitor node
        g_monitor = std::make_shared<TopicMonitor>();
        
        // Create display
        g_display = std::make_unique<NCursesDisplay>(g_monitor);
        
        // Start ROS2 spinning in a separate thread
        std::thread ros_thread([&]() {
            rclcpp::spin(g_monitor);
        });
        
        // Give the monitor more time to discover initial topics and let ROS2 graph stabilize
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Run the display in the main thread
        g_display->run();
        
        // Cleanup
        g_running = false;
        rclcpp::shutdown();
        
        if (ros_thread.joinable()) {
            ros_thread.join();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    return 0;
}