# synBio
ROS2 Development for Laboratory Automation utilizing Robotic Manipulators


## TO DO LIST:

### Planning and Manipulation Tasks
- [ ] Create package for UR operation
  - [ ] Create launch files to operate UR Dashboard
- [x] Create package / nodes to communicate with Unity scene
  - [ ] Compare point

### Computer Vision Tasks
- [x] Create package for Realsense operation
  - [ ] Processing image
- [ ] Create pointcloud set with image overlay of recognizable positions
- [ ] Process pointcloud ros bag file (`/home/rs_recording_529/rs_recording_529_0.db3`)

### Digital Twin / Unity Tasks
- [ ] Create package for UR description and digital twin mesh / URDF files

---
# LAYOUT:

### Workspace Test:
- URDF configuration files (table, camera, dual arms)
- launch file for Rviz display
### Synbio Test:
- Unity launch file
- Python files for
  - Fixed frame broadcaster (Camera TF to world)
  - Realsense reading (Checks for green object)
  - Sending GPIO (Zimmer Gripper operation)
### Zimmer Description
- URDF files (xacro, macro) for custom end effector
- Launch file for display