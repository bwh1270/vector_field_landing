#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import csv
import serial
import numpy as np
import pandas as pd
from collections import deque

import rospy
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import PoseStamped, PointStamped
from aims_als.msg import Marker

'''
Gimbal Control Class
Function:
    - Request: send the desired angle to gimbal through packet
    - Receive: get the angle from gimbal

Caution:
    - First request and then receive. So, it can be said fast phase system.
    - Our Gimbal rotates Rz(psi) -> Rx(phi) -> Ry(theta) following the installed motor position. 

Wiki:
    - A cyclic redundancy check (CRC) is an error-detecting code commonly used in digital networks 
      and storage devices to detect accidental changes to digital data.
        
'''


class GimbalControl:
    def __init__(self):
        
        self.init_roll_ = 0
        self.init_tilt_ = 0
        self.init_pan_ = 0
        self.g_pan_before_init = True
        self.g_pan_init_ = 0
        self.b_pan_init_ = 0

        self.detected_ = False

        # Frame: {W}, {B}, {G}, {S}, {C}, {M}
        self.pW_B_ = np.array([0,0,0])
        self.qW_B_ = np.array([1,0,0,0])
        self.R_WB_ = np.array([[1,0,0],
                               [0,1,0],
                               [0,0,1]])
        self.uav_dq = deque()
        self.uav_dq.append([0, self.pW_B_, self.R_WB_])

        self.tB_S_ = np.array([0.06, 0, -0.09])  # tB_S
        self.R_WG_ = np.array([[1, 0,0],
                               [0,-1,0],
                               [0,0,-1]]) 

        self.R_GS_dq = deque()

        self.R_SC_ = np.array([[0,0,1],
                               [1,0,0],
                               [0,1,0]])

        self.pC_M_ = np.array([0,0,0])
        self.R_CM_ = np.array([[1,0,0],
                               [0,1,0],
                               [0,0,1]])
        self.img_time = 0
        self.img_header = Header()

        self.pG_M_ = np.array([0,0,0])

        self.phi_des_old_ = 0
        self.theta_des_old_ = 0


        self.init_cmd_ = [
            
            [92, 110, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

            [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

            [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

            [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

            [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

            [92, 110, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 뭔지는 파악 안됨 아마 run gimbal 일듯

            [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

            [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 100, 200, 100, 
            0, 0, 10, 138, 90, 166, self.init_roll_, self.init_tilt_, self.init_pan_, 40, 35, 40, 1, 1, 1, 0, 0, 0, 0, 0, 1, 
            0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

            [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 30, 80, 0, 15, 70, 10, 
            20, 30, 0, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
            0, 0]
        ]

        self.default_cmd_ = [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # For analysis about delay reduction
        # self.fields_ = ['time', 'reference', 'response', 'command']
        self.fields_ = ['time', 'roll_des', 'roll', 'roll_cmd', 
                                'pitch_des','pitch', 'pitch_cmd']
        self.data_ = []

        self.first_loop_ = True
        self.t0_ = 0.0

        # Parameters
        self._loop_rate = rospy.get_param('~loop_rate',  50.0)
        _port = rospy.get_param('~serial_port', '/dev/gimbal')
        self._init_pitch = rospy.get_param('~init_pitch', 0.0)
        _baud_rate = rospy.get_param('~baud_rate',     115200)
        self._debug = rospy.get_param('~debug',         False)

        _vfov = rospy.get_param('/vfov', 43.75)
        _pitch_margin_ratio = rospy.get_param('~pitch_margin_ratio', 0.5)
        self.vfov_margin = np.deg2rad(_vfov*_pitch_margin_ratio)

        # Open serial port
        try:
            self.ser_ = serial.Serial(_port, _baud_rate, timeout=1)
            rospy.loginfo(f"Opened serial port: {_port} at {_baud_rate} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port {_port}: {e}")
            raise

        # Set Loop Rate
        self.rate_ = rospy.Rate(self._loop_rate) 

        # ROS
        self.pub_transformed_marker_ = rospy.Publisher('/aims/transformed_marker', PoseStamped, queue_size=1)
        self.pub_gimbal_angle_ = rospy.Publisher('/aims/gimbal_angles', PointStamped, queue_size=1)

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.uavPoseCb, queue_size=1)
        rospy.Subscriber("/aims/fractal_detections", Marker, self.markerCb, queue_size=1)


    def computeCRC(self, data):
        '''
        Compute the CRC value for the input data using the CRC-16-CCITT algorithm.
        This value is error detection during data request.
        '''
        crc = 0x0000
        polynomial = 0x1021
        for byte in data:
            crc ^= (int(byte) << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def splitCRC(self, crc):
        '''
        Split the 16-bit(2-byte) CRC value into upper 8 bits and lower 8 bits.
        This is used to send the CRC value as a byte when transmitting data.
        '''
        high_byte = (crc >> 8) & 0xFF
        low_byte = crc & 0xFF
        return high_byte, low_byte

    def appendCRC(self, data):
        '''
        Compute CRC value on input data and add it to the end of data. 
        This is done mandatory before data request.
        '''
        if len(data) != 62:
            raise ValueError("Input list must contain exactly 62 decimal values.")
        crc = self.computeCRC(data)
        high_byte, low_byte = self.splitCRC(crc)
        return data + [high_byte, low_byte]

    def transmitData(self, data):
        '''
        Transmit data through serial port after convering data to byte format.
        '''
        bit_string = ''.join(format(int(byte), '08b') for byte in data)
        byte_data = bytearray(int(bit_string[i:i+8], 2) for i in range(0, len(bit_string), 8))
        self.ser_.write(byte_data)

    def computeAngle(self, byte_H, byte_L):
        '''
        Combines the upper byte and lower byte to compute the angles of the gimbal(roll,tilt,pan).
        '''
        angle = (byte_H << 8) | byte_L
        if angle & 0x8000:
            angle = angle - 0x10000
        return angle / 100

    def E2q(self, E: list):
        E_np = np.array(E) * np.pi / 180
        q = np.array([
            np.cos(E[0]/2.)*np.cos(E[1]/2.)*np.cos(E[2]/2.) + np.sin(E[0]/2.)*np.sin(E[1]/2.)*np.sin(E[2]/2.),
            np.sin(E[0]/2.)*np.cos(E[1]/2.)*np.cos(E[2]/2.) - np.cos(E[0]/2.)*np.sin(E[1]/2.)*np.sin(E[2]/2.),
            np.cos(E[0]/2.)*np.sin(E[1]/2.)*np.cos(E[2]/2.) + np.sin(E[0]/2.)*np.cos(E[1]/2.)*np.sin(E[2]/2.),
            np.cos(E[0]/2.)*np.cos(E[1]/2.)*np.sin(E[2]/2.) - np.sin(E[0]/2.)*np.sin(E[1]/2.)*np.cos(E[2]/2.)
        ])
        return q

    def receive(self):
        '''
        Reads 64 bytes of data from the serial port and calculates roll, tilt, and pan angles.
        Publishes the calculated angle values ​​as ROS messages.
        '''
        if self.ser_.in_waiting >= 64:
            data_from_ser = self.ser_.read(64)
            data_bits_ser = ''.join(format(byte, '08b') for byte in data_from_ser)
            data_list_ser = [int(data_bits_ser[i:i+8], 2) for i in range(0, len(data_bits_ser), 8)]

            '''
            Gimbal frame{G}:
                - g1 == b2 (uav nose)
                - g2 == b1 (uav right)
                - g3 == -b3 (uav down)
            ''' 
            roll = self.computeAngle(data_list_ser[7], data_list_ser[6])
            tilt = self.computeAngle(data_list_ser[9], data_list_ser[8])
            pan = self.computeAngle(data_list_ser[11], data_list_ser[10])
            
            return roll, tilt, pan
        return None, None, None

    def requestInit(self, request_iter):
        '''
        Takes a command from the init_cmd_ list, adds a CRC, and transmits it.
        Used during the gimbal initialization process.
        If you don't, gimbal does not move.
        '''
        cmd = self.init_cmd_[request_iter] if request_iter < len(self.init_cmd_) else self.default_cmd_
        cmd_with_crc = self.appendCRC(cmd)
        self.transmitData(cmd_with_crc)
        return request_iter + 1

    def terminateCtrl(self):
        '''
        Sends a command to terminate gimbal control.
        Called when the program terminates.
        '''
        ter_cmd = [92, 110, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        cmd_with_crc = self.appendCRC(ter_cmd)
        self.transmitData(cmd_with_crc)

    def request(self, roll, tilt, pan):
        '''
        Generates a command based on the input roll, tilt, and pan angles, adds a CRC, and then transmits it.
        Used to control the gimbal to a specific angle.
        [  pos  ][  neg  ]
        '''
        roll = 256 + roll if roll < 0 else roll
        tilt = 256 + tilt if tilt < 0 else tilt
        pan  = 256 + pan if pan < 0 else pan
        cmd = [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 100, 200, 100, 0, 0, 10, 138, 90, 166, roll, tilt, pan, 40, 30, 40, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        cmd_with_crc = self.appendCRC(cmd)
        self.transmitData(cmd_with_crc)
    
    def q2E(selef, q: np.ndarray) -> np.ndarray:
        qq = q ** 2
        E = np.zeros(3)
        E[0] = np.arctan2(2. * (q[2] * q[3] + q[0] * q[1]), (qq[0] + qq[3] - qq[1] - qq[2]))
        E[1] = -np.arcsin(2. * (q[1] * q[3] - q[0] * q[2]))
        E[2] = np.arctan2(2. * (q[1] * q[2] + q[0] * q[3]), (qq[0] + qq[1] - qq[2] - qq[3]))
        return E

    def R2E(self, R: np.ndarray) -> np.ndarray:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = -np.arcsin(R[2, 0])  
        yaw = np.arctan2(R[1, 0], R[0, 0])
        return np.array([roll, pitch, yaw])
    
    def E2R(self, roll, pitch, yaw):
        # 각도 -> 라디안 (필요하면)
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        
        # ZYX 순서로 회전행렬 만들기
        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]
        ])
        return R

    def gimbalE2R(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        c_r, s_r = np.cos(roll), np.sin(roll)
        c_p, s_p = np.cos(pitch), np.sin(pitch)
        c_y, s_y = np.cos(yaw), np.sin(yaw)

        R_x = np.array([[1, 0, 0],
                        [0, c_r, -s_r],
                        [0, s_r, c_r]])

        R_y = np.array([[c_p, 0, s_p],
                        [0, 1, 0],
                        [-s_p, 0, c_p]])

        R_z = np.array([[c_y, -s_y, 0],
                        [s_y, c_y, 0],
                        [0, 0, 1]])

        # Combine the rotations: R = Rz * Rx * RY
        return (R_z @ R_x) @ R_y


    def q2R(self, q: np.ndarray) -> np.ndarray:
        q = q / np.linalg.norm(q)
        R = np.zeros((3, 3))
        qq = q ** 2

        R[0, 0] = qq[0] + qq[1] - qq[2] - qq[3]
        R[0, 1] = 2. * (q[1] * q[2] - q[0] * q[3])
        R[0, 2] = 2. * (q[1] * q[3] + q[0] * q[2])

        R[1, 0] = 2. * (q[1] * q[2] + q[0] * q[3])
        R[1, 1] = qq[0] + qq[2] - qq[1] - qq[3]
        R[1, 2] = 2. * (q[2] * q[3] - q[0] * q[1])

        R[2, 0] = 2. * (q[1] * q[3] - q[0] * q[2])
        R[2, 1] = 2. * (q[2] * q[3] + q[0] * q[1])
        R[2, 2] = qq[0] + qq[3] - qq[1] - qq[2]

        return R


    def R2q(self, R: np.ndarray) -> np.ndarray:
        q = np.zeros(4)
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[0] = 0.25 / s
            q[1] = (R[2, 1] - R[1, 2]) * s
            q[2] = (R[0, 2] - R[2, 0]) * s
            q[3] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[0] = (R[2, 1] - R[1, 2]) / s
                q[1] = 0.25 * s
                q[2] = (R[0, 1] + R[1, 0]) / s
                q[3] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[0] = (R[0, 2] - R[2, 0]) / s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = 0.25 * s
                q[3] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[0] = (R[1, 0] - R[0, 1]) / s
                q[1] = (R[0, 2] + R[2, 0]) / s
                q[2] = (R[1, 2] + R[2, 1]) / s
                q[3] = 0.25 * s

        return q

    def uavPoseCb(self, msg):
        self.pW_B_ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.qW_B_ = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        self.R_WB_ = self.q2R(self.qW_B_)
        
        self.uav_dq.append([msg.header.stamp.to_sec(), self.pW_B_, self.R_WB_])
        if (len(self.uav_dq) >= 10):
            self.uav_dq.popleft()

    def get_yaw_only_projection(self, R):
        # Extract yaw angle (Z-Y-X convention)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        
        # Construct yaw-only rotation matrix
        R_proj = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0,            0,           1]
        ])
    
        return R_proj

    def remove_roll(self, R):
        # Extract pitch and yaw from rotation matrix (Z-Y-X order)
        theta = np.arcsin(-R[2, 0])  # pitch
        psi = np.arctan2(R[1, 0], R[0, 0])  # yaw
        
        # Reconstruct R_proj = Rz(yaw) * Ry(pitch)
        cy, sy = np.cos(psi), np.sin(psi)
        cp, sp = np.cos(theta), np.sin(theta)

        R_proj = np.array([
            [cy * cp, -sy, cy * sp],
            [sy * cp,  cy, sy * sp],
            [-sp,     0,    cp]
        ])
        
        return R_proj

    def markerCb(self, msg):
        detected = msg.detect
        if (detected):
            self.pC_M_ = np.array([msg.pose_stmp.pose.position.x, msg.pose_stmp.pose.position.y, msg.pose_stmp.pose.position.z])
            qC_M_ = np.array([msg.pose_stmp.pose.orientation.w, msg.pose_stmp.pose.orientation.x, msg.pose_stmp.pose.orientation.y, msg.pose_stmp.pose.orientation.z])
            self.R_CM_ = self.q2R(qC_M_)
            # self.R_CM_ = self.remove_roll(self.R_CM_)

            self.detected_ = True
            self.img_header = msg.pose_stmp.header
            self.img_time = msg.pose_stmp.header.stamp.to_sec()
            # print(self.img_time)


        else:
            self.detected_ = False
            return

    def clamp(self, x, lower_bound, upper_bound):
        x = np.rad2deg(x)
        return int(round(max(lower_bound, min(upper_bound, x))))

    def computeDesiredAngle(self, gimbal_roll, gimbal_pitch, epsilon=1e-9):

        # Compute the desired angle
        phi_des = np.arctan2(-self.pG_M_[1], self.pG_M_[2])
        theta_des = np.arctan2(-self.pG_M_[2], self.pG_M_[0]*np.cos(phi_des))

        ## *** Step input ***
        # theta_des = np.deg2rad(-90)

        ## *** Ramp input ***
        # if (self.first_loop_):
        #     self.t0_ = rospy.Time.now().to_sec()
        #     self.first_loop_ = False
        # slope = 45.0
        # theta_des = -(rospy.Time.now().to_sec() - self.t0_) * slope
        # if (theta_des < -90):
        #     theta_des = -90 
        # theta_des = np.deg2rad(theta_des)

        ## *** Sinusodial input ***
        # if (self.first_loop_):
        #     self.t0_ = rospy.Time.now().to_sec()
        #     self.first_loop_ = False
        # t = rospy.Time.now().to_sec() - self.t0_
        # phi_des = 0 + (-20)*np.sin(t * np.pi / 2.)
        # theta_des = -45 + (-45)*np.cos(t * np.pi / 2.)
        # phi_des = np.deg2rad(phi_des)
        # theta_des = np.deg2rad(theta_des)
        
        # if (t >= 10.0):
        #     phi_des = 0
        #     theta_des = 0

        
        # Additional Controller (adding the zero if error is larger than margin)
        #                        no control action otherwise
        err_pitch = np.abs(gimbal_pitch - theta_des)  # radian
 
        if (err_pitch >= self.vfov_margin):
            K_theta = 22
            theta_cmd = theta_des + (theta_des - self.theta_des_old_) * K_theta
            # K_phi   = 55 
            # phi_cmd = phi_des + (phi_des - self.phi_des_old_) * K_phi
        else:
            theta_cmd = gimbal_pitch

        # Clamping
        phi_cmd = 0  #self.clamp(phi_cmd, -25, 25)
        theta_cmd = self.clamp(theta_cmd, -100, -5)  # real limit[-115,10]

        # @debug
        # print(theta_des, self.theta_des_old_, theta_cmd)
        # self.data_.append([rospy.Time.now().to_sec(), np.rad2deg(phi_des), gimbal_roll, phi_cmd, np.rad2deg(theta_des), gimbal_pitch, theta_cmd])

        # Update
        # self.phi_des_old_ = phi_des
        self.theta_des_old_ = theta_des

        return phi_cmd, theta_cmd
    

    def run(self):
        request_iter = 0
        ref_roll, ref_tilt, ref_pan = 0, self._init_pitch, 0
        ctrl_loop_iter = 0

        try:
            while not rospy.is_shutdown():

                if (ctrl_loop_iter == 500):
                    ctrl_loop_iter = 500
                else:
                    print(f'Wait for stabilized {ctrl_loop_iter/500*100}%')
                    ctrl_loop_iter += 1
                    continue
                
                gimbal_roll, gimbal_tilt, gimbal_pan = self.receive()
                if (self._debug):
                    if gimbal_roll is None or gimbal_tilt is None or gimbal_pan is None:
                        print("[ERROR] Gimbal data not received properly.")
                    else:
                        print(f"[Roll,Tilt,Pan]=[{gimbal_roll:.2f},{gimbal_tilt:.2f},{gimbal_pan:.2f}]")

                if request_iter < len(self.init_cmd_):
                    request_iter = self.requestInit(request_iter)
                    
                elif request_iter == len(self.init_cmd_):
                    request_iter += 1
                    request_iter = self.requestInit(request_iter)
                    self.g_pan_init_ = gimbal_pan 
                    self.b_pan_init_ = np.rad2deg(self.R2E(self.R_WB_)[-1])
                    print("Gimbal initialization complete !!")

                else:
                    print(f'des: {ref_roll}')
                    print(f'res: {gimbal_roll}')

                    self.request(ref_roll, ref_tilt, ref_pan)

                    if (gimbal_roll is not None) and (self.detected_):

                        # Gimbal control

                        # # @dynamic calibration for yaw
                        # # print(abs(ref_tilt - gimbal_tilt))
                        # if ((self.g_pan_before_init) and (abs(ref_tilt - gimbal_tilt) <= 3)) :
                        #     print(f'First: {self.g_pan_init_}')
                        #     print(f'Update: {gimbal_pan}')
                        #     self.g_pan_init_ = gimbal_pan 
                        #     self.g_pan_before_init = False


                        # dt_lst = [abs(rospy.Time.now().to_sec() - x[0]) for x in list(self.uav_dq)]
                        # kdx = dt_lst.index(min(dt_lst))
                        # R_WB = self.uav_dq[kdx][-1]
                        # b_pan = np.rad2deg(self.R2E(R_WB)[-1])
                        # # print(self.b_pan_init_)
                        # # print(b_pan)
                        # # print(self.g_pan_init_ - (b_pan - self.b_pan_init_))
                        # # print(gimbal_pan)
                        # gimbal_pan = gimbal_pan - (self.g_pan_init_ - (b_pan - self.b_pan_init_))

                        # R_GS = self.gimbalE2R(np.deg2rad(gimbal_roll), np.deg2rad(gimbal_tilt), np.deg2rad(gimbal_pan))
                        R_GS = self.gimbalE2R(np.deg2rad(gimbal_roll), np.deg2rad(gimbal_tilt), 0.)
                        

                        self.R_GS_dq.append([rospy.Time.now().to_sec(), R_GS])
                        if (len(self.R_GS_dq) >= 10):
                            self.R_GS_dq.popleft()

                        # @debug
                        # print(f"UAV p=[{self.pW_B_[0]:.2f}, {self.pW_B_[1]:.2f}, {self.pW_B_[2]:.2f}")
                        # print(f"UAV R=", self.R_WB_);
                        # print(f"Marker p=[{self.pC_M_[0]:.2f}, {self.pC_M_[1]:.2f}, {self.pC_M_[2]:.2f}")
                        # print(f"Marker R=", self.R_CM_);
                        # print(f"Gimbal R=", R_GS);
                        
                        dt_lst = [abs(self.img_time - x[0]) for x in list(self.R_GS_dq)]
                        idx = dt_lst.index(min(dt_lst))
                        # print(f"Gimbal: (idx, dt) = ({idx}/10), {min(dt_lst):.4f})")

                        # @debug
                        # print(np.array(dt_lst))
                        # print(f'delay: [{dt_lst[idx]-dt_lst[-1]}]')
               
                        self.pG_M_ = self.R_GS_dq[idx][-1] @ (self.R_SC_ @ self.pC_M_) 

                        phi_des, theta_des = self.computeDesiredAngle(np.deg2rad(gimbal_roll), np.deg2rad(gimbal_tilt)) 
                        ref_roll = phi_des
                        ref_tilt = theta_des

                        # @debug
                        # print(f"Gimbal: [Roll,Tilt,Pan]=[{gimbal_roll:.2f},{gimbal_tilt:.2f},{gimbal_pan:.2f}]")
                        # print(f"[desired] = {phi_des}, {theta_des}")
                        # self.data_.append([rospy.Time.now().to_sec(), ref_tilt, gimbal_tilt])

                        dt_lst = [abs(self.img_time - x[0]) for x in list(self.uav_dq)]
                        jdx = dt_lst.index(min(dt_lst))
                        # print(f"UAV: (idx, dt) = ({jdx}/10), {min(dt_lst):.4f})")

                        # Transformation Marker
                        # print(idx, jdx)
                        R_GS = self.R_GS_dq[idx][-1]
                        R_WB = self.uav_dq[jdx][-1]
                        pW_B = self.uav_dq[jdx][1]

                        # E = self.R2E(self.R_WB_)
                        # print(f"UAV: [Roll,Tilt,Pan]=[{np.rad2deg(E[0]):.2f},{np.rad2deg(E[1]):.2f},{np.rad2deg(E[2]):.2f}]")
                        # pW_M = R_WB @ (self.R_BG_ @ (R_GS @ (self.R_SC_ @ self.pC_M_)) + self.tB_G_) + pW_B
                        # qW_M = self.R2q(R_WB @ (self.R_WG_ @ (R_GS @ (self.R_SC_ @ self.R_CM_))))

                        R_WB_yaw = self.E2R(0., 0., self.R2E(R_WB)[-1])
                        pW_M = R_WB_yaw @ (self.R_WG_ @ (R_GS @ (self.R_SC_ @ self.pC_M_))) + pW_B + R_WB @ self.tB_S_
                        qW_M = self.R2q(R_WB_yaw @ (self.R_WG_ @ (R_GS @ (self.R_SC_ @ self.R_CM_))))

                        # if (ctrl_loop_iter % 100 == 0):
                        # #     self.g_pan_init_ = gimbal_pan
                        #     ctrl_loop_iter = 0
                        #     print(f"pC_M: {self.pC_M_}")
                        #     print(f"pS_M: {self.R_SC_ @ self.pC_M_}")
                        #     print(f"pG_M: {R_GS @ (self.R_SC_ @ self.pC_M_)}")
                        #     print(f"pW_M: {R_WB_yaw @ (self.R_WG_ @ (R_GS @ (self.R_SC_ @ self.pC_M_)))}")
                        #     print(f'position: {pW_B + R_WB @ self.tB_S_}')

                        # print(f"pW_M: {pW_M}")

                        # @debug
                        # E = self.q2E(qW_M)
                        
                        transformed_marker = PoseStamped()
                        transformed_marker.header = self.img_header
                        transformed_marker.pose.position.x = pW_M[0]
                        transformed_marker.pose.position.y = pW_M[1]
                        transformed_marker.pose.position.z = pW_M[2]
                        transformed_marker.pose.orientation.w = qW_M[0]
                        transformed_marker.pose.orientation.x = qW_M[1]
                        transformed_marker.pose.orientation.y = qW_M[2]
                        transformed_marker.pose.orientation.z = qW_M[3]
                        self.pub_transformed_marker_.publish(transformed_marker)
                        
                        gimbal_angles = PointStamped()
                        gimbal_angles.header = transformed_marker.header
                        gimbal_angles.point.x = gimbal_roll
                        gimbal_angles.point.y = gimbal_tilt
                        gimbal_angles.point.z =  self.g_pan_init_ - gimbal_pan
                        self.pub_gimbal_angle_.publish(gimbal_angles)
                        
                        # update
                        # if (ctrl_loop_iter % 100 == 0):
                        #     self.g_pan_init_ = gimbal_pan
                        #     ctrl_loop_iter = 0

                    
                self.rate_.sleep()

        # except:
        #     print("Some error occurs !!")
        #     pass

        finally:
            # @debug
            # with open('test.csv', 'w',newline='') as f:  
            #     write = csv.writer(f) 
            #     write.writerow(self.fields_) 
            #     write.writerows(self.data_)
            
            self.terminateCtrl()
            self.ser_.close()
            
            print("Shutting down...")


if __name__ == '__main__':
    rospy.init_node('gimbal_ctrl_node', anonymous=True)
    gimbal_control = GimbalControl()
    gimbal_control.run()
