# Procedure to Develop a New Package
- [x] Develop on a host machine
- [x] Copy the package over to voxl-nodes/catkin_ws/src on the host machine
- [x] Launch the voxl_emulator_docker
- [x] Build the package in that docker
- [x] Get back to the host machine, make_package.sh
- [x] On the host machine, connect Voxl board to the host machine using an USB cable,
 and  install_on_voxl.sh 

# Procedure to launch the high resolution cam ROS node on the voxl board  
* For some more details, refer to https://docs.modalai.com/voxl-cam-ros/
- [x] ssh to the board,
- [x] launch file is in 
```
/opt/ros/indigo/share/voxl_cam_ros/launch/hires_color.launch
```
- [x] Modify this launch file to ensure the camera ID is correct. 
Check the HIRES_CAM_ID
- [x] Launch
```
roslaunch voxl_cam_ros hires.launch 
```  
