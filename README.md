## first-time setup

Prerequisites: ROS 2 Jazzy, MoveIt 2, and Gazebo (Harmonic) already installed
and sourced (e.g. in your `~/.bashrc`).

1. Clone this repo into the `src/` folder of a colcon workspace:
   ```
   mkdir -p ~/<WORKSPACE_NAME>/src
   cd ~/<WORKSPACE_NAME>/src
   git clone https://github.com/viaductlights/robot_modelling.git
   ```
2. Build the whole workspace from the workspace root (not from inside this
   repo):
   ```
   cd ~/<WORKSPACE_NAME>
   colcon build
   ```
3. Source the workspace:
   ```
   source install/setup.bash
   ```
4. Bring everything up with the bringup script (see below), or use the
   individual launch commands further down this file.

## bringup script

`task_coordinator/scripts/bringup.sh` automates the kill-and-restart routine
used throughout development: it kills any leftover sim/`move_group`/
`hmi_motion_server`/HMI processes from a previous run, launches
`iss_moveit_gz_sim.launch.py` (Gazebo + both move_groups + rviz), detaches
the capsule from bean before anything can move (the `DetachableJoint`
plugin has no "start detached" option - it auto-attaches at world load),
waits for both robot controllers to come up, then starts
`hmi_motion_server` and the HMI GUI.

Run it from anywhere (it auto-detects the workspace root from its own
location):
```
bash src/robot_modelling/task_coordinator/scripts/bringup.sh
```

It logs each component to `/tmp/sim_bringup_<timestamp>.log`,
`/tmp/hmi_motion_server_<timestamp>.log`, and `/tmp/hmi_gui_<timestamp>.log`,
and prints the log paths plus a final status line once everything is ready.

## sim_task script

`task_coordinator/scripts/run_sim_task.sh` automates running the docking
demo end-to-end without the HMI: it kills any leftover sim/`move_group`
instances, launches `iss_moveit_gz_sim.launch.py`, waits for both robot
controllers to come up, then runs `task_coordinator sim_task` in the
foreground so you can watch the bean -> goal1 -> attach -> goal2 -> nemo ->
goal3 sequence execute live. `sim_task` detaches the capsule from bean
itself as its first action, so the script doesn't need to do that
separately. It doesn't start `hmi_motion_server` or the HMI GUI -
`sim_task` drives both robots directly with its own `MoveGroupInterface`s,
so those would just be redundant clients.

```
bash src/robot_modelling/task_coordinator/scripts/run_sim_task.sh
```

Gazebo/`move_group`/rviz are left running after `sim_task` finishes so you
can inspect the result; rerun the script for a clean restart.

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
