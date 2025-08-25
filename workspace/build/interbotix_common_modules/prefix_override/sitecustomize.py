import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/emackinnon1/InterbotixControlServer/workspace/install/interbotix_common_modules'
