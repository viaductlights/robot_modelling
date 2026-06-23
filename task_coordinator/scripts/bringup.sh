#!/usr/bin/env bash
# Kills any previous sim/move_group/hmi_motion_server/hmi instances and
# starts a clean stack: Gazebo + both move_groups + rviz (via the
# iss_moveit_gz_sim launch file), detaches the capsule from bean before
# anything can move (DetachableJoint has no "start detached" option - it
# auto-attaches at world load), then starts hmi_motion_server and the HMI GUI.
set -o pipefail

# Defaults to the colcon workspace root this script lives under
# (.../<ws>/src/robot_modelling/task_coordinator/scripts/bringup.sh -> <ws>),
# so it works regardless of where the workspace is checked out. Override with
# WS_DIR=... if your workspace root isn't 4 levels up from this script.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
LOG_DIR="${LOG_DIR:-/tmp}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SIM_LOG="${LOG_DIR}/sim_bringup_${STAMP}.log"
HMI_SERVER_LOG="${LOG_DIR}/hmi_motion_server_${STAMP}.log"
HMI_GUI_LOG="${LOG_DIR}/hmi_gui_${STAMP}.log"

echo "==> Killing any existing sim/move_group/hmi_motion_server/hmi processes"
pkill -9 -f "ros2 launch iss_simulation" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "move_group" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "ros_gz_bridge" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "hmi_motion_server" 2>/dev/null
pkill -9 -f "ros2 run hmi hmi" 2>/dev/null
sleep 2

remaining="$(pgrep -af "gz sim|move_group|hmi_motion_server|robot_state_publisher|rviz2|ros2 launch|ros_gz_bridge|ros2 run hmi hmi" 2>/dev/null)"
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

echo "==> Waiting for /bean/detach topic to come up"
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q "^/bean/detach$"; then
    echo "    found after ${i}s"
    break
  fi
  sleep 1
done

echo "==> Detaching capsule from bean before anything can move"
timeout 5 ros2 topic pub --once /bean/detach std_msgs/msg/Empty "{}" >/dev/null

echo "==> Waiting for both robot_controllers to activate"
for i in $(seq 1 60); do
  count="$(grep -c "Successfully switched controllers" "${SIM_LOG}" 2>/dev/null || true)"
  if [ "${count:-0}" -ge 4 ]; then
    echo "    controllers active after ${i}s"
    break
  fi
  sleep 1
done

echo "==> Starting hmi_motion_server (log: ${HMI_SERVER_LOG})"
nohup bash -c "source install/setup.bash && ros2 run task_coordinator hmi_motion_server" \
  > "${HMI_SERVER_LOG}" 2>&1 &
disown

for i in $(seq 1 15); do
  if grep -q "Services ready" "${HMI_SERVER_LOG}" 2>/dev/null; then
    break
  fi
  sleep 1
done

echo "==> Starting HMI GUI (log: ${HMI_GUI_LOG})"
nohup bash -c "source install/setup.bash && ros2 run hmi hmi" \
  > "${HMI_GUI_LOG}" 2>&1 &
disown
sleep 4

echo "==> Status"
grep -n "Services ready\|ERROR" "${HMI_SERVER_LOG}" 2>/dev/null
cat "${HMI_GUI_LOG}" 2>/dev/null
echo "    sim log:         ${SIM_LOG}"
echo "    hmi_server log:  ${HMI_SERVER_LOG}"
echo "    hmi_gui log:     ${HMI_GUI_LOG}"
