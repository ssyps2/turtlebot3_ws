import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ruize/Desktop/ShuCao_object_follower/install/camera_viewer'
