import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alumno1/Downloads/TP_Final_Robotica-Localizing/install/tpf'
