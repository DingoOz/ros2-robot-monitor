#include "ros2_robot_monitor/ncurses_display.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <thread>
#include <chrono>
#include <iterator>
#include <locale.h>

NCursesDisplay::NCursesDisplay(std::shared_ptr<TopicMonitor> monitor)
    : monitor_(monitor), stdscr_(nullptr), chart_win_(nullptr), running_(true), 
      show_chart_(false), selected_row_(0), scroll_offset_(0) {
}

NCursesDisplay::~NCursesDisplay() {
    cleanup_display();
}

void NCursesDisplay::init_display() {
    // Set locale for Unicode support
    setlocale(LC_ALL, "");
    
    stdscr_ = initscr();
    if (!stdscr_) {
        throw std::runtime_error("Failed to initialize ncurses");
    }
    
    noecho();
    cbreak();
    curs_set(0);
    keypad(stdscr_, TRUE);
    nodelay(stdscr_, TRUE);
    timeout(100);
    
    init_colors();
}

void NCursesDisplay::init_colors() {
    if (has_colors()) {
        start_color();
        init_pair(1, COLOR_GREEN, COLOR_BLACK);   // Active topics / Accel X
        init_pair(2, COLOR_RED, COLOR_BLACK);     // Inactive topics / Accel Y
        init_pair(3, COLOR_YELLOW, COLOR_BLACK);  // Headers / Accel Z
        init_pair(4, COLOR_CYAN, COLOR_BLACK);    // Selected row / Gyro X
        init_pair(5, COLOR_MAGENTA, COLOR_BLACK); // Frequency / Gyro Y
        init_pair(6, COLOR_BLUE, COLOR_BLACK);    // Gyro Z
        init_pair(7, COLOR_WHITE, COLOR_BLACK);   // Default/Labels
    }
}

void NCursesDisplay::cleanup_display() {
    if (chart_win_) {
        delwin(chart_win_);
        chart_win_ = nullptr;
    }
    if (stdscr_) {
        nocbreak();
        keypad(stdscr_, FALSE);
        echo();
        endwin();
        stdscr_ = nullptr;
    }
}

void NCursesDisplay::draw_header() {
    int height, width;
    getmaxyx(stdscr_, height, width);
    
    // Title
    std::string title = "ROS2 Topic Monitor";
    int title_x = (width - title.length()) / 2;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, title_x, "%s", title.c_str());
    attroff(COLOR_PAIR(3) | A_BOLD);
    
    // Timestamp
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream timestamp;
    timestamp << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    std::string timestamp_str = timestamp.str();
    int timestamp_x = (width - timestamp_str.length()) / 2;
    attron(COLOR_PAIR(3));
    mvprintw(1, timestamp_x, "%s", timestamp_str.c_str());
    attroff(COLOR_PAIR(3));
    
    // Column headers
    attron(COLOR_PAIR(3) | A_UNDERLINE);
    mvprintw(3, 0, "%-30s %-25s %-10s %-30s", "Topic", "Type", "Freq (Hz)", "Data");
    attroff(COLOR_PAIR(3) | A_UNDERLINE);
    
    // Instructions
    std::string instructions = "UP/DOWN: navigate, 'q': quit, 'r': refresh, 'c': toggle IMU chart";
    if (static_cast<int>(instructions.length()) < width && height > 4) {
        attron(COLOR_PAIR(3));
        mvprintw(height - 1, 0, "%s", instructions.c_str());
        attroff(COLOR_PAIR(3));
    }
}

void NCursesDisplay::draw_topics() {
    int height, width;
    getmaxyx(stdscr_, height, width);
    
    int start_row = 4;
    int display_height = height - 5;
    
    auto topic_data = monitor_->get_topic_data();
    
    // Convert to vector and sort by topic name only
    std::vector<std::pair<std::string, TopicInfo>> topics;
    for (const auto& [name, info] : topic_data) {
        topics.emplace_back(name, info);
    }
    std::sort(topics.begin(), topics.end(), 
              [](const auto& a, const auto& b) { return a.first < b.first; });
    
    // Handle selection bounds
    if (selected_row_ >= static_cast<int>(topics.size())) {
        selected_row_ = std::max(0, static_cast<int>(topics.size()) - 1);
    }
    
    // Handle scrolling
    if (selected_row_ < scroll_offset_) {
        scroll_offset_ = selected_row_;
    } else if (selected_row_ >= scroll_offset_ + display_height) {
        scroll_offset_ = selected_row_ - display_height + 1;
    }
    
    // Draw topics
    for (int i = 0; i < display_height && i + scroll_offset_ < static_cast<int>(topics.size()); ++i) {
        int topic_idx = i + scroll_offset_;
        const auto& [topic_name, topic_info] = topics[topic_idx];
        
        int row = start_row + i;
        int color_pair = 0;
        int attributes = 0;
        
        // Determine color and attributes
        if (topic_idx == selected_row_) {
            color_pair = 4;
            attributes = A_REVERSE;
        } else if (topic_info.is_active) {
            color_pair = 1;
        } else {
            color_pair = 2;
        }
        
        // Truncate strings to fit columns
        std::string topic_short = topic_name.length() > 29 ? topic_name.substr(0, 29) : topic_name;
        std::string type_short = topic_info.type;
        if (type_short.find("msg/") != std::string::npos) {
            type_short = type_short.substr(type_short.find("msg/") + 4);
        }
        if (type_short.length() > 24) {
            type_short = type_short.substr(0, 24);
        }
        
        std::ostringstream freq_stream;
        freq_stream << std::fixed << std::setprecision(1) << topic_info.frequency;
        std::string freq_str = freq_stream.str();
        
        std::string data_str = topic_info.last_data;
        if (data_str.length() > 29) {
            data_str = data_str.substr(0, 29);
        }
        if (data_str.empty()) {
            data_str = "No data";
        }
        
        // Draw the row
        attron(COLOR_PAIR(color_pair) | attributes);
        mvprintw(row, 0, "%-30s %-25s %-10s %-30s", 
                 topic_short.c_str(), type_short.c_str(), freq_str.c_str(), data_str.c_str());
        attroff(COLOR_PAIR(color_pair) | attributes);
        
        // Clear rest of line
        clrtoeol();
    }
}

bool NCursesDisplay::handle_input() {
    int key = getch();
    
    if (key == 'q' || key == 'Q') {
        return false;
    }
    
    auto topic_data = monitor_->get_topic_data();
    int topic_count = static_cast<int>(topic_data.size());
    
    switch (key) {
        case KEY_UP:
            if (show_chart_) {
                // Navigate between IMU topics
                auto imu_topics = monitor_->get_imu_topics();
                if (!imu_topics.empty()) {
                    auto it = std::find(imu_topics.begin(), imu_topics.end(), selected_imu_topic_);
                    if (it != imu_topics.begin()) {
                        selected_imu_topic_ = *(--it);
                    }
                }
            } else {
                selected_row_ = std::max(0, selected_row_ - 1);
            }
            break;
        case KEY_DOWN:
            if (show_chart_) {
                // Navigate between IMU topics
                auto imu_topics = monitor_->get_imu_topics();
                if (!imu_topics.empty()) {
                    auto it = std::find(imu_topics.begin(), imu_topics.end(), selected_imu_topic_);
                    if (it != imu_topics.end() && ++it != imu_topics.end()) {
                        selected_imu_topic_ = *it;
                    }
                }
            } else {
                selected_row_ = std::min(topic_count - 1, selected_row_ + 1);
            }
            break;
        case 'r':
        case 'R':
            // Force immediate topic discovery
            monitor_->discover_topics();
            break;
        case 'c':
        case 'C':
            // Toggle IMU chart window
            {
                auto imu_topics = monitor_->get_imu_topics();
                if (!imu_topics.empty()) {
                    show_chart_ = !show_chart_;
                    if (show_chart_ && selected_imu_topic_.empty()) {
                        selected_imu_topic_ = imu_topics[0];
                    }
                }
            }
            break;
        default:
            break;
    }
    
    return true;
}

void NCursesDisplay::run() {
    init_display();
    
    try {
        while (running_) {
            clear();
            
            if (show_chart_) {
                draw_imu_chart();
            } else {
                draw_header();
                draw_topics();
            }
            
            refresh();
            
            if (!handle_input()) {
                break;
            }
            
            // Small delay to prevent excessive CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (const std::exception& e) {
        cleanup_display();
        throw;
    }
    
    cleanup_display();
}

void NCursesDisplay::draw_imu_chart() {
    int height, width;
    getmaxyx(stdscr_, height, width);
    
    // Chart header
    std::string title = "IMU Data Chart - " + selected_imu_topic_;
    int title_x = (width - title.length()) / 2;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, title_x, "%s", title.c_str());
    attroff(COLOR_PAIR(3) | A_BOLD);
    
    // Instructions
    std::string instructions = "'c': back to topics, 'q': quit, UP/DOWN: select IMU topic";
    attron(COLOR_PAIR(3));
    mvprintw(height - 1, 0, "%s", instructions.c_str());
    attroff(COLOR_PAIR(3));
    
    // Get IMU data
    auto imu_data = monitor_->get_imu_data(selected_imu_topic_);
    if (imu_data.empty()) {
        mvprintw(height / 2, (width - 20) / 2, "No IMU data available");
        return;
    }
    
    // Extract data vectors for charting
    std::vector<double> acc_x, acc_y, acc_z;
    std::vector<double> gyro_x, gyro_y, gyro_z;
    
    for (const auto& data : imu_data) {
        acc_x.push_back(data.acc_x);
        acc_y.push_back(data.acc_y);
        acc_z.push_back(data.acc_z);
        gyro_x.push_back(data.gyro_x);
        gyro_y.push_back(data.gyro_y);
        gyro_z.push_back(data.gyro_z);
    }
    
    // Draw charts (each taking about height/6 rows) with different colors
    int chart_height = (height - 4) / 6;
    draw_text_chart(stdscr_, acc_x, "Linear Acc X", 2, chart_height, 1);  // Green
    draw_text_chart(stdscr_, acc_y, "Linear Acc Y", 2 + chart_height, chart_height, 2);  // Red
    draw_text_chart(stdscr_, acc_z, "Linear Acc Z", 2 + 2*chart_height, chart_height, 3);  // Yellow
    draw_text_chart(stdscr_, gyro_x, "Angular Vel X", 2 + 3*chart_height, chart_height, 4);  // Cyan
    draw_text_chart(stdscr_, gyro_y, "Angular Vel Y", 2 + 4*chart_height, chart_height, 5);  // Magenta
    draw_text_chart(stdscr_, gyro_z, "Angular Vel Z", 2 + 5*chart_height, chart_height, 6);  // Blue
}

void NCursesDisplay::draw_text_chart(WINDOW* win, const std::vector<double>& data, const std::string& title, int start_y, int height, int color_pair) {
    if (data.empty() || height < 3) return;
    
    int width;
    getmaxyx(win, height, width);
    
    // Chart title with chart color
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    mvprintw(start_y, 0, "%s", title.c_str());
    attroff(COLOR_PAIR(color_pair) | A_BOLD);
    
    // Find min/max for scaling
    auto minmax = std::minmax_element(data.begin(), data.end());
    double min_val = *minmax.first;
    double max_val = *minmax.second;
    
    if (min_val == max_val) {
        min_val -= 1.0;
        max_val += 1.0;
    }
    
    // Chart area dimensions
    int chart_width = std::min(static_cast<int>(data.size()), width - 15);
    int chart_height_actual = height - 2;
    
    if (chart_width <= 0 || chart_height_actual <= 0) return;
    
    // Draw Y-axis labels with subtle color
    attron(COLOR_PAIR(7));
    mvprintw(start_y + 1, 0, "%6.2f", max_val);
    mvprintw(start_y + 1 + chart_height_actual/2, 0, "%6.2f", (min_val + max_val) / 2);
    mvprintw(start_y + 1 + chart_height_actual, 0, "%6.2f", min_val);
    attroff(COLOR_PAIR(7));
    
    // Draw the filled area chart with simple reliable characters
    for (int x = 0; x < chart_width; ++x) {
        int data_idx = data.size() - chart_width + x;
        
        if (data_idx >= 0 && data_idx < static_cast<int>(data.size())) {
            double normalized = (data[data_idx] - min_val) / (max_val - min_val);
            double chart_value = normalized * chart_height_actual;
            
            // Calculate which row the data point falls in
            int data_row = static_cast<int>(chart_value);
            
            // Clamp data_row to valid range
            if (data_row >= chart_height_actual) {
                data_row = chart_height_actual - 1;
            }
            if (data_row < 0) {
                data_row = 0;
            }
            
            // Fill from bottom up to the data point
            for (int y = 0; y < chart_height_actual; ++y) {
                int screen_y = start_y + 1 + chart_height_actual - 1 - y;
                
                if (y <= data_row) {
                    // Fill with colored character based on position
                    attron(COLOR_PAIR(color_pair));
                    
                    // Use different ASCII characters for better visual effect
                    if (y == data_row) {
                        // Top of the fill - use a different character for sub-pixel precision
                        double fraction = chart_value - data_row;
                        if (fraction > 0.75) {
                            mvprintw(screen_y, 8 + x, "#");  // Almost full
                        } else if (fraction > 0.5) {
                            mvprintw(screen_y, 8 + x, "=");  // Upper half
                        } else if (fraction > 0.25) {
                            mvprintw(screen_y, 8 + x, "-");  // Lower half
                        } else {
                            mvprintw(screen_y, 8 + x, ".");  // Small bottom
                        }
                    } else {
                        // Full fill below data point
                        mvprintw(screen_y, 8 + x, "#");
                    }
                    
                    attroff(COLOR_PAIR(color_pair));
                } else {
                    // Empty space above data point
                    mvprintw(screen_y, 8 + x, " ");
                }
            }
        } else {
            // No data for this column, fill with spaces
            for (int y = 0; y < chart_height_actual; ++y) {
                int screen_y = start_y + 1 + chart_height_actual - 1 - y;
                mvprintw(screen_y, 8 + x, " ");
            }
        }
    }
}