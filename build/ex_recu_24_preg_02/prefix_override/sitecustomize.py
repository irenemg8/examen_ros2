import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/irene/examen/examen_ros2/install/ex_recu_24_preg_02'
