import os
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG)
if PKG not in sys.path:
    sys.path.insert(0, PKG)
os.environ.setdefault('UR5_PLATFORM_CONFIG', os.path.join(PKG, 'config', 'platform.yaml'))
