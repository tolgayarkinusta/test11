"""Validate system setup and dependencies."""
import os
import sys
from pymavlink import mavutil

def check_mavlink():
    """Test MAVLink connection."""
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        master.wait_heartbeat(timeout=5)
        print('MAVLink bağlantısı başarılı')
        master.close()
        return True
    except Exception as e:
        print(f'MAVLink bağlantı hatası: {e}')
        return False

def check_model():
    """Check if model file exists."""
    if not os.path.exists('balonLarge54.pt'):
        print('Model dosyası eksik')
        return False
    print('Model dosyası mevcut')
    return True

def main():
    """Run all validation checks."""
    success = True
    
    print('\n=== Python Sürümü ===')
    print(f'Python {sys.version}')
    
    print('\n=== MAVLink Testi ===')
    if not check_mavlink():
        success = False
    
    print('\n=== Model Dosyası Kontrolü ===')
    if not check_model():
        success = False
    
    print('\n=== CUDA Kontrolü ===')
    import check_cuda
    if not check_cuda.check_cuda():
        success = False
    
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
