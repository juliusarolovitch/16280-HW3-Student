# 16280 Planning

## Setup Instructions

You will need to review the interface for AmenablePDDL, see it here: https://github.com/juliusarolovitch/Amenable-PDDL/blob/main/README.md

1. **Necessary code** for you to fill in the following scripts:  
   - `planner_node.py`
  

2. **Build the required packages** using:  
   ```sh
   colcon build --packages-select image_processing_pkg object_detection_interfaces
   ```  

3. **Source the setup file**:  
   ```sh
   source install/setup.bash
   ```  

## Running Object Detection & Planning

To perform planning using TurtleBot images:  

1. **Start the camera node** on your TurtleBot:  
   _(Ensure the TurtleBot is powered on and the camera node is running in a separate terminal.)_  

2. **Launch the detection pipeline** on your VM:  
   ```sh
   ros2 launch image_processing_pkg detection_pipeline_launch.py
   ```  

This will initiate the planning process using images captured by the TurtleBot.
