import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user1/ros2_ws_2502/install/modelrobot_pkg_nea'
