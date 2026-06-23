#!/usr/bin/env bash
# Kills any previous sim/move_group instances and runs the full docking demo
# end-to-end: launches Gazebo + both move_groups + rviz (via the
# iss_moveit_gz_sim launch file), waits for both robot controllers to come
# up, then runs `task_coordinator sim_task` in the foreground so you can
# watch the bean -> goal1 -> attach -> goal2 -> nemo -> goal3 sequence
# execute live. sim_task detaches the capsule from bean itself as its first
# action (DetachableJoint has no "start detached" option - it auto-attaches
# at world load), so this script doesn't need to do that separately.
#
# Does NOT start hmi_motion_server or the HMI GUI - sim_task drives bean and
# nemo directly with its own MoveGroupInterfaces, so those aren't needed
# (and would just be redundant clients). Gazebo/move_group/rviz are left
# running after sim_task finishes so you can inspect the result; rerun this
# script for a clean restart.
set -o pipefail

# Defaults to the colcon workspace root this script lives under
# (.../<ws>/src/robot_modelling/task_coordinator/scripts/run_sim_task.sh ->
# <ws>), so it works regardless of where the workspace is checked out.
# Override with WS_DIR=... if your workspace root isn't 4 levels up from
# this script.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
LOG_DIR="${LOG_DIR:-/tmp}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SIM_LOG="${LOG_DIR}/sim_bringup_${STAMP}.log"

echo "==> Killing any existing sim/move_group/sim_task processes"
pkill -9 -f "ros2 launch iss_simulation" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "move_group" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "ros_gz_bridge" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "task_coordinator sim_task" 2>/dev/null
sleep 2

remaining="$(pgrep -af "gz sim|move_group|robot_state_publisher|rviz2|ros2 launch|ros_gz_bridge|sim_task" 2>/dev/null)"
if [ -n "${remaining}" ]; then
  echo "WARNING: some processes did not die, you may need to kill them manually:"
  echo "${remaining}"
fi

cd "${WS_DIR}" || exit 1
source install/setup.bash

echo "==> Launching iss_moveit_gz_sim.launch.py (log: ${SIM_LOG})"
nohup bash -c "source install/setup.bash && ros2 launch iss_simulation iss_moveit_gz_sim.launch.py" \
  > "${SIM_LOG}" 2>&1 &
disown

echo "==> Waiting for both robot_controllers to activate"
for i in $(seq 1 60); do
  count="$(grep -c "Successfully switched controllers" "${SIM_LOG}" 2>/dev/null || true)"
  if [ "${count:-0}" -ge 4 ]; then
    echo "    controllers active after ${i}s"
    break
  fi
  sleep 1
done

echo "==> Running sim_task (live output below)"
echo "    sim log: ${SIM_LOG}"
ros2 run task_coordinator sim_task
