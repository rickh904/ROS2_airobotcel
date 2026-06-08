import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/workspace/src/ROS2_airobotcel/install/voice_control_stijn'
