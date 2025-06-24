# IMU Chart Demo Instructions

## 🚀 How to Test the Braille Charts

### 1. Start the Dynamic IMU Publisher
```bash
cd /home/dingo/Programming/ros2-robot-monitor
python3 test_imu_publisher.py
```

### 2. Run the Monitor (in a separate terminal)
```bash
cd /home/dingo/Programming/ros2-robot-monitor
source install/setup.bash
ros2 run ros2_robot_monitor ros2_monitor
```

### 3. Toggle to Chart View
- You should see the `/imu` topic listed at 20.0 Hz
- Press **'c'** to switch to the braille chart view
- You'll see 6 beautiful high-resolution charts showing:
  - **Linear Acc X** (Green) - sinusoidal pattern
  - **Linear Acc Y** (Red) - cosine pattern  
  - **Linear Acc Z** (Yellow) - gravity + oscillation
  - **Angular Vel X** (Cyan) - slow sine wave
  - **Angular Vel Y** (Magenta) - medium cosine wave
  - **Angular Vel Z** (Blue) - fast sine wave

### 4. Chart Features
- **High Resolution**: Each character can show 4 vertical levels using braille dots
- **Real-time Updates**: 20Hz refresh rate
- **10-second History**: Rolling window of IMU data
- **Auto-scaling**: Charts automatically adjust to data range
- **Color-coded**: Each axis has its own distinct color

### 5. Navigation
- **UP/DOWN arrows**: Switch between different IMU topics (if multiple available)
- **'c'**: Toggle back to topic list
- **'q'**: Quit the application

## 🎨 Chart Visualization
The charts use braille characters (U+2800–U+28FF) for ultra-high resolution:
- Empty: ⠀ (U+2800)
- Partial fill patterns using individual dots
- Full: ⣿ (U+28FF)

Each character can represent 4 different vertical levels, giving much finer detail than traditional block characters!