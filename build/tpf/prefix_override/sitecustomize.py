import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/franny/Documentos/UdeSA/TpFinalRobotica/TP_Final_Robotica/install/tpf'
