from pymavlink import mavutil
import math
import cv2
import numpy
import math
import time


def map_value_to_range(value, in_min=-1, in_max=1, out_min=-2000, out_max=2000):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


class USVController:
    def __init__(self, connection_string):
        # connecting to the vehicle
        print("Heartbeat waiting...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("Heartbeat found!")

        self.first_position = None
        self.current_position = None

    def arm_vehicle(self):
        # arming vehicle
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("Waiting for the vehicle to arm...")
        self.master.motors_armed_wait()  # Waiting for arming the vehicle
        print('Armed!')

    def disarm_vehicle(self):
        # Disarming the vehicle
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("Waiting for the vehicle to disarm")
        self.master.motors_disarmed_wait()  # Waiting for disarming the vehicle
        print('Disarmed!')

    def print_motor_outputs(self):
        print(self.master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True))

    def go_to_vehicle_raw(self, x=0, y=0, z=500, r=0, buttons=0, s=0, t=0, see_motor_output=0):
        # Manual controls
        self.master.mav.manual_control_send(
            self.master.target_system,
            x,  # x is straight 2000 is full front -2000 full back
            y,  # y is sideways 2000 is full right -2000 full left
            z,  # z is up 1500 is full up -500 full down (pwm is 1100 up 1900 down all z axis motors)
            r,
            buttons,
            s,
            t
        )
        if see_motor_output:
            self.print_motor_outputs()

    def set_servo(self, servo_pin, pwm_value, see_motor_output=0):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_pin,
            pwm_value,
            0, 0, 0, 0, 0
        )
        if see_motor_output:
            self.print_motor_outputs()

    def set_mode(self, mode_name):
        # (example: 'STABILIZE', 'MANUAL', 'DEPTH_HOLD')

        mode_id = self.master.mode_mapping()[mode_name]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print("Aracın modu " + mode_name + " olarak değiştirildi.")

   







