import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bachelor/ros2_ws/src/v1_topic_pkg/install/v1_topic_pkg'
