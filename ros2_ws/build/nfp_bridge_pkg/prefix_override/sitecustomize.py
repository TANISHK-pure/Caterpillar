import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tanishk/ros2_ws/install/nfp_bridge_pkg'
