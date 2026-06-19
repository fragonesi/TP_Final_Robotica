import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/romanella/Documents/ros2_ws/src/TP_Final_Robotica/install/tpf'
