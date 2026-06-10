#!/usr/bin/env python3

# main.py
import sys
import rclpy
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from hmi_gui import RobotCellGUI
from hmi_bridge import HmiBridge

def main(args=None):
    # 1. Initialiseer ROS2 en PyQt Context
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # 2. Maak instanties van de GUI en de ROS2 Bridge
    gui = RobotCellGUI()
    bridge = HmiBridge()

    # 3. KOPPELING: Van ROS2 Bridge data -> Naar de GUI updates
    bridge.ros_image_received.connect(gui.set_camera_image)
    bridge.ros_state_received.connect(gui.set_system_state)
    bridge.ros_counts_received.connect(gui.set_product_counts)

    # 4. KOPPELING: Van GUI Knoppen -> Naar ROS2 Acties
    gui.start_pressed.connect(lambda: bridge.publish_command("START"))
    gui.stop_pressed.connect(lambda: bridge.publish_command("STOP"))
    gui.reset_pressed.connect(lambda: bridge.publish_command("RESET"))
    gui.home_pressed.connect(bridge.trigger_home_service)
    gui.confidence_changed.connect(bridge.update_ai_parameter)

    # 5. De ROS2 Executor Timer
    # Dit zorgt ervoor dat ROS2 elke 10ms events verwerkt, midden in de PyQt loop
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(bridge, timeout_sec=0))
    ros_timer.start(10)

    # 6. Open de HMI
    gui.show()

    # 7. Zorg voor een nette shutdown bij het sluiten van de applicatie
    exit_code = app.exec_()
    
    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()