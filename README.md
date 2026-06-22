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

### sim task, after ^ launch  
`ros2 run task_coordinator sim_task"

#### note  
need to write single launchfile for sim!  

### HMI + motion server, w/ moveit
build custom interfaces + backend + HMI:  
`colcon build --packages-select task_coordinator hmi`  

after the moveit bringup above is running, start the backend:  
`ros2 run task_coordinator hmi_motion_server`  

then launch the HMI:  
`ros2 run hmi hmi`  

#### services exposed by hmi_motion_server
- `/bean/go_home`, `/nemo/go_home` (`std_srvs/Trigger`)  
- `/bean/select_pose`, `/nemo/select_pose` (`task_coordinator/SelectPose`) - moves directly to a named hardcoded pose  
- `/bean/move_to_pose`, `/nemo/move_to_pose` (`task_coordinator/MoveToPose`) - arbitrary Cartesian target, planned via MoveIt IK  

call them directly for testing, e.g.:  
`ros2 service call /bean/select_pose task_coordinator/srv/SelectPose "{pose_name: pose_1}"`  
`ros2 service call /bean/go_home std_srvs/srv/Trigger`  

#### note
bean and nemo run on separate MoveIt callback groups, so go_home/select_pose/move_to_pose
for one robot can run concurrently with the other - they only serialize against
their own robot's other calls.  

#### speeding up sim for testing

`gz service -s /world/orbit/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --req 'max_step_size: 0.01, real_time_factor: 0'`  

## in progress:  
- combined launchfile  
- pose2 (bean), pose3+ (nemo)  
- presentation
