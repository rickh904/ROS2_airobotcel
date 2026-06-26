#!/usr/bin/env bash

echo "=========================================="
echo " ROBOTCEL RESET SCRIPT GESTART"
echo "=========================================="

LOG_FILE="/tmp/robotcel_restart.log"

echo "[RESET] $(date) Reset gestart" >> "$LOG_FILE"

# Eerst ROS omgeving sourcen zodat ros2 topic pub werkt
cd ~/ROS2_airobotcel || exit 1

source /opt/ros/jazzy/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/depthai_env/bin/activate
source install/setup.bash

# Kleine vertraging zodat HMI-knop callback netjes kan afronden
sleep 1

echo "[RESET] STOP publiceren..." >> "$LOG_FILE"

ros2 topic pub --once /system/stop_request std_msgs/msg/Bool "{data: true}" >> "$LOG_FILE" 2>&1 || true

sleep 1

echo "[RESET] Oude robotcel processen stoppen..." >> "$LOG_FILE"

# Launches stoppen
pkill -9 -f "robotcel.launch.py" || true
pkill -9 -f "real_robot.launch.py" || true
pkill -9 -f "vision.launch.py" || true

# Robot / MoveIt / controllers stoppen
pkill -9 -f "ros2_control_node" || true
pkill -9 -f "controller_manager" || true
pkill -9 -f "spawner" || true
pkill -9 -f "move_group" || true
pkill -9 -f "rviz2" || true
pkill -9 -f "xarm_driver_node" || true
pkill -9 -f "robot_state_publisher" || true
pkill -9 -f "joint_state_publisher" || true

# Eigen nodes stoppen
pkill -9 -f "main_controller" || true
pkill -9 -f "robot_manipulator" || true
pkill -9 -f "hmi_node" || true
pkill -9 -f "voice_node" || true
pkill -9 -f "positie_transformatie" || true
pkill -9 -f "vision_node" || true
pkill -9 -f "ai_vision_node" || true

sleep 2

echo "[RESET] ROS daemon resetten..." >> "$LOG_FILE"

ros2 daemon stop >> "$LOG_FILE" 2>&1 || true
sleep 1
ros2 daemon start >> "$LOG_FILE" 2>&1 || true
sleep 2

echo "[RESET] Robotcel opnieuw starten..." >> "$LOG_FILE"

# Nieuwe terminal openen als dat kan.
# Als gnome-terminal niet beschikbaar is, start hij de launch op de achtergrond.
if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal -- bash -lc "
        cd ~/ROS2_airobotcel
        source /opt/ros/jazzy/setup.bash
        source ~/xarm_ws/install/setup.bash
        source ~/depthai_env/bin/activate
        source install/setup.bash
        ros2 launch robotcel_bringup robotcel.launch.py
        exec bash
    "
else
    nohup bash -lc "
        cd ~/ROS2_airobotcel
        source /opt/ros/jazzy/setup.bash
        source ~/xarm_ws/install/setup.bash
        source ~/depthai_env/bin/activate
        source install/setup.bash
        ros2 launch robotcel_bringup robotcel.launch.py
    " >> "$LOG_FILE" 2>&1 &
fi

echo "[RESET] Nieuwe launch gestart" >> "$LOG_FILE"

echo "=========================================="
echo " RESET SCRIPT KLAAR"
echo "=========================================="
