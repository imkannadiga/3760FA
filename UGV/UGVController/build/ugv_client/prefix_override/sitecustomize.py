import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/softdcar/3760FA/UGV/UGVController/install/ugv_client'
