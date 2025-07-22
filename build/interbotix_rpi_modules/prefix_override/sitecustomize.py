import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/emackinnon1/interbotix_ws/install/interbotix_rpi_modules'
