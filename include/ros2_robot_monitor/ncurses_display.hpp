#pragma once

#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <chrono>

#include "topic_monitor.hpp"

#undef OK
#include <ncurses.h>

struct ImuDataPoint {
    std::chrono::steady_clock::time_point timestamp;
    double acc_x, acc_y, acc_z;
    double gyro_x, gyro_y, gyro_z;
};

class NCursesDisplay {
public:
    NCursesDisplay(std::shared_ptr<TopicMonitor> monitor);
    ~NCursesDisplay();
    
    void init_display();
    void cleanup_display();
    void run();
    
private:
    std::shared_ptr<TopicMonitor> monitor_;
    WINDOW* stdscr_;
    WINDOW* chart_win_;
    bool running_;
    bool show_chart_;
    int selected_row_;
    int scroll_offset_;
    std::deque<ImuDataPoint> imu_history_;
    std::string selected_imu_topic_;
    
    void draw_header();
    void draw_topics();
    void draw_imu_chart();
    void update_imu_data();
    bool handle_input();
    void init_colors();
    std::vector<std::string> get_imu_topics();
    void draw_text_chart(WINDOW* win, const std::vector<double>& data, const std::string& title, int start_y, int height, int color_pair);
};