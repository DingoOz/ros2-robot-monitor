#pragma once

#include <string>
#include <vector>
#include <memory>

#include "topic_monitor.hpp"

#undef OK
#include <ncurses.h>

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
    bool running_;
    int selected_row_;
    int scroll_offset_;
    
    void draw_header();
    void draw_topics();
    bool handle_input();
    void init_colors();
};