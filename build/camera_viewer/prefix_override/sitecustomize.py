import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pengyuan/Desktop/turtlebot3_ws/install/camera_viewer'
