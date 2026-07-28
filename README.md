# vive_ros2_workspace


This repository is the ROS 2 workspace for the HTC Vive tracking system. 
It has been enhanced to support multiple trackers with advanced stability features.

## Key Features & Contributions
* **Multi-Tracker Support**: Capable of dynamically identifying and tracking up to 6 Vive Trackers simultaneously to meet autonomous vehicle positioning requirements.
* **Auto-Reconnection Mechanism**: Automatically runs a background reconnection routine if a tracker loses signal due to interference, ensuring seamless data recovery.


## Configuration (Include How to add your Tracker IDs)
Since each Vive Tracker has a unique hardware serial number, you must configure your tracker IDs before compiling(will be better)

### Clone and Installation (For First-time Setup)
If you are setting up this workspace on a new computer or autonomous vehicle, follow these steps to clone and install dependencies:
     

     # 1. clone the repo
     git clone git@github.com:hrc-pme/vive_ros2.git
     cd vive_ros2

     # 2. build the Docker Image
     ./build.sh
     # (or exec: docker build -t vive_tracker_humble:latest .)

     # 3. open container
     ./run.sh

     # 4. Build the ROS 2 workspace (Inside Container - First time only)
     colcon build --symlink-install
     source install/setup.bash



## OpenVR & Tracker Communication Setup

1. Open the core input source file:
   ```bash
   code src/vive_ros2/src/vive_input.cpp

   # Example configuration in vive_input.cpp:
   if (tracker_sn == "LHR-D6B44530") { role_index = 0; } 
   else if (tracker_sn == "LHR-481141A3") { role_index = 1; } 
   else if (tracker_sn == "LHR-2E8F88BF") { role_index = 2; }
   # Add more trackers here if needed...
   #you can check the tracker id here : http://localhost:27062/console/index.html

2. After modifying C++ files, you must rebuild the package inside the Docker container:
   ```bash
   colcon build --symlink-install
   source install/setup.bash

## Start your workspace（ Docker)
1. Terminal 1:
   ```bash
   ros2 run vive_ros2 vive_input

2. Terminal 2:
   ```bash
   ros2 run vive_ros2 vive_node 100

3. Terminal 3: Check your tracker
   ```bash
   ros2 topic echo /vive_pose_abs
   # can check which one tracker you need
   # ros2 topic echo /tracker1/vive_pose_abs

4. Terminal 4: Open rosbridge for conenct to unity
   ```bash
   #check your IP Address first
   hostname -I
   cd unity_ws/src
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml

5. Terminal 5: Recording ROSBAG
   ```bash
   #Method 1: Using the provided helper script (Recommended)
   #Inside the Docker container terminal, simply execute:
   ./record_vive.sh
   
   #Method 2: Standard ROS 2 Command
   ros2 bag record -o <bag_name> /tf /tf_static /<your_topic>
