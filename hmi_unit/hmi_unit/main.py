#!/usr/bin/env python3

# main.py
import sys
import signal  # NIEUW: Nodig om Ctrl+C op te vangen
import rclpy
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from hmi_unit.hmi_gui import RobotCellGUI
from hmi_unit.hmi_bridge import HmiBridge

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
    bridge.ros_error_received.connect(gui.add_error_message)

    # 4. KOPPELING: Van GUI Knoppen & Sliders -> Naar ROS2 Backend
    gui.start_pressed.connect(bridge.trigger_start_sort)      
    gui.stop_pressed.connect(bridge.trigger_stop_sort)        
    gui.reset_pressed.connect(bridge.trigger_reset_service)    
    gui.home_pressed.connect(bridge.trigger_home_action)       
    gui.confidence_changed.connect(bridge.update_ai_parameter)
    gui.speed_changed.connect(bridge.publish_speed)

    # 5. De ROS2 Executor Timer
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(bridge, timeout_sec=0))
    ros_timer.start(10)

    # 6. NIEUW: Ctrl+C handler toevoegen zodat terminal sluiting forceert
    # Dit dwingt PyQt5 om af te sluiten zodra het OS een SIGINT stuurt
    signal.signal(signal.SIGINT, lambda sig, frame: app.quit())

    # 7. Open de HMI
    gui.show()

    # 8. Zorg voor een nette shutdown bij het sluiten van de applicatie
    exit_code = app.exec_()
    
    # Netjes opruimen na sluiten (of na Ctrl+C)
    print("\n[HMI] Bezig met afsluiten van de ROS2 node...")
    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()