import os
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG)
SRC = os.path.join(PKG, 'src')
if SRC not in sys.path:
    sys.path.insert(0, SRC)
os.environ.setdefault('UR5_PLATFORM_CONFIG', os.path.join(PKG, 'config', 'platform.yaml'))
