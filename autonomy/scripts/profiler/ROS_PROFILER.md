# ROS Profiler Demo - Fixed Instructions

The demo has been fixed to resolve the `rospy.init_node()` error. Here are the updated instructions:

## Option 1: Using Launch File (Recommended)

### Terminal 1: Start all demo nodes
```bash
cd /home/catkin_ws/src/hydrus-software-stack/autonomy/scripts/services
./run_profiler_demo.sh
# Choose option 1 (Run demo nodes with launch file)
```

### Terminal 2: Run the profiler
```bash
cd /home/catkin_ws/src/hydrus-software-stack/autonomy/scripts/services
python3 ros_profiler.py --nodes image_processor data_analyzer demo_camera --export demo_profile.csv
```

## Option 2: Manual Node Startup

### Terminal 1: Camera node
```bash
cd /home/catkin_ws/src/hydrus-software-stack/autonomy/scripts/services
python3 ros_profiler_demo.py camera
```

### Terminal 2: Processor node
```bash
cd /home/catkin_ws/src/hydrus-software-stack/autonomy/scripts/services
python3 ros_profiler_demo.py processor
```

### Terminal 3: Analyzer node
```bash
cd /home/catkin_ws/src/hydrus-software-stack/autonomy/scripts/services
python3 ros_profiler_demo.py analyzer
```

### Terminal 4: Run profiler
```bash
cd /home/catkin_ws/src/hydrus-software-stack/autonomy/scripts/services
python3 ros_profiler.py --nodes image_processor data_analyzer demo_camera --export demo_profile.csv
```

## What was fixed:

1. **Removed multiple `rospy.init_node()` calls** - Each node class no longer calls `rospy.init_node()` in its constructor
2. **Added command-line arguments** - You can now run individual nodes: `camera`, `processor`, or `analyzer`
3. **Created launch file** - `profiler_demo.launch` properly starts all nodes in separate processes
4. **Updated launcher script** - The shell script now offers the launch file option

## Quick Test:

1. Run: `./run_profiler_demo.sh` and choose option 1
2. In another terminal, run: `python3 ros_profiler.py --export demo_profile.csv`
3. You should see the real-time profiling dashboard!

The error has been resolved - each node now runs in its own process with its own `rospy.init_node()` call.
