#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_topic_discovery");
    
    std::cout << "Waiting for ROS2 graph to stabilize..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    auto topics = node->get_topic_names_and_types();
    
    std::cout << "Found " << topics.size() << " topics:" << std::endl;
    for (const auto& [name, types] : topics) {
        std::cout << "  " << name << ":";
        for (const auto& type : types) {
            std::cout << " " << type;
        }
        std::cout << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
}