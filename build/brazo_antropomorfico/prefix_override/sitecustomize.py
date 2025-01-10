import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/monica/ros2_brazo/install/brazo_antropomorfico'
