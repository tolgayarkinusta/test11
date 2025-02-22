"""
Test dependencies and setup requirements for the USV control system.
Verifies CUDA, camera, MAVLink and model availability.
"""
import os
import subprocess

from pyzed import sl
import torch
import cv2
import numpy
import ultralytics
from pymavlink import mavutil


def check_cuda():
    """Check if CUDA is available and properly configured."""
    cuda_available = torch.cuda.is_available()
    print(f'CUDA available: {cuda_available}')
    if not cuda_available:
        raise RuntimeError('CUDA is not available but required')


def check_camera():
    """Check if ZED camera is connected and accessible."""
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                capture_output=True, text=True, check=True)
        if 'ZED' not in result.stdout:
            raise RuntimeError('ZED camera not found')
        print('ZED camera detected')

        # Test camera connection
        cam = sl.Camera()
        init_params = sl.InitParameters()
        status = cam.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'ZED camera initialization failed: {status}')
        print('ZED camera initialization successful')
        cam.close()
    except Exception as e:
        print(f'Camera check error: {e}')
        raise


def check_model():
    """Check if YOLO model file exists and can be loaded."""
    try:
        model_path = 'balonLarge54.pt'
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'{model_path} model not found')
        print('YOLO model file found')

        # Try loading model with CUDA
        model = ultralytics.YOLO(model_path)
        print('YOLO model loaded successfully')
    except Exception as e:
        print(f'Model check error: {e}')
        raise


def check_mavlink():
    """Check MAVLink connection and basic functionality."""
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        print('Waiting for heartbeat...')
        master.wait_heartbeat(timeout=5)
        print('MAVLink heartbeat received')
        master.close()
    except Exception as e:
        print(f'MAVLink check error: {e}')
        raise


def check_packages():
    """Verify that all required Python packages are installed."""
    required = ['torch', 'cv2', 'numpy', 'ultralytics', 'pymavlink']
    for pkg in required:
        try:
            __import__(pkg)
        except ImportError as e:
            raise ImportError(
                f'Required package {pkg} is not installed: {e}') from e

    # Special check for pyzed since it needs different import
    try:
        from pyzed import sl
        print('All required Python packages are installed')
    except ImportError as e:
        raise ImportError('Required package pyzed is not installed') from e


if __name__ == '__main__':
    check_packages()
    check_cuda()
    check_camera()
    check_model()
    check_mavlink()
