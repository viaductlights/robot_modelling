## working spawns:  

### individually, in rviz, w/o moveit: 
`ros2 launch bean3_description bean3_rviz.launch.py`  
`ros2 launch nemo1_description nemo1_rviz.launch.py`  

### individually, in rviz and gz, w/ moveit  
`ros2 launch iss_simulation bean3_gz_test.launch.py`  
`ros2 launch iss_simulation nemo1_gz_test.launch.py`  

### together, in gz, w/o moveit:  
`ros2 launch iss_simulation iss_gz_sim.launch.py`  

### together, in rviz and gz, w/ moveit  
`ros2 launch iss_simulation iss_moveit_gz_sim.launch.py`  

#### note  
need to manually set bean's controllers when switching from nemo to bean (and possibly vice versa)  
check controller status:  
`ros2 control list_controllers --controller-manager /bean/controller_manager`  
set if robot_controller or joint_state_broadcaster as needed:  
`ros2 control set_controller_state robot_controller active --controller-manager /bean/controller_manager`  

## in progress:  
phase 0 of sim  

