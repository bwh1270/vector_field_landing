#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time as time_module
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
import numpy as np
from collections import deque
from geometry_msgs.msg import Pose
import math


filtered_apriltag_buffer = deque() # Apriltag data 수집
window_size = 5 # fps 계산에 사용(data 개수)
buffer_time = 3 # buffer 저장 시간 구간 지정(s)
fps_pub = None  # main()에서 초기화

ini_roll = 0
ini_tilt = 0
ini_pan = 0

initial_command = [

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
0, 0, 10, 138, 90, 166, ini_roll, ini_tilt, ini_pan, 40, 35, 40, 1, 1, 1, 0, 0, 0, 0, 0, 1, 
0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

[92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 30, 80, 0, 15, 70, 10, 
20, 30, 0, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0]]


default_command = [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# CRC 계산 및 데이터 처리 함수들 정의
def crc16_ccitt_zero(data):
    crc = 0x0000
    polynomial = 0x1021
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def split_crc(crc):
    high_byte = (crc >> 8) & 0xFF
    low_byte = crc & 0xFF
    return high_byte, low_byte

def append_crc_to_data(data):
    if len(data) != 62:
        raise ValueError("입력 리스트는 정확히 62개의 10진수 값을 포함해야 합니다.")
    
    crc = crc16_ccitt_zero(data)
    high_byte, low_byte = split_crc(crc)
    
    data_with_crc = data + [high_byte, low_byte]
    return data_with_crc

def send_data(data, ser_A):
    bit_string = ''.join(format(byte, '08b') for byte in data)
    byte_data = bytearray(int(bit_string[i:i+8], 2) for i in range(0, len(bit_string), 8))
    ser_A.write(byte_data)

def calculate_angle(byte_H, byte_L):
    angle = (byte_H << 8) | byte_L
    if angle & 0x8000:
        angle = angle - 0x10000
    return angle / 100

def process_receive(ser_A, angle_pub):

    if ser_A.in_waiting >= 64:
        data_from_A = ser_A.read(64)
        angle_msg = Float64MultiArray()
        
        data_bits_A = ''.join(format(byte, '08b') for byte in data_from_A)
        data_list_A = [int(data_bits_A[i:i+8], 2) for i in range(0, len(data_bits_A), 8)]
        
        roll = calculate_angle(data_list_A[7], data_list_A[6])
        tilt = calculate_angle(data_list_A[9], data_list_A[8])
        pan = calculate_angle(data_list_A[11], data_list_A[10])
        
        #print(f"roll: {roll}, tilt: {tilt}, pan: {pan}")
        angle_msg.data = [roll, tilt, pan]
        angle_pub.publish(angle_msg)
        
        return roll, tilt, pan
    else:
        return None, None, None

def process_initial_request(ser_A, command_index):
    
    if command_index < len(initial_command):
        command = initial_command[command_index]
    else:
        command = default_command

    command_with_crc = append_crc_to_data(command)
    send_data(command_with_crc, ser_A)

    return (command_index + 1)

def process_finish(ser_A):
    finish_command = [92, 110, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]   
    
    command_with_crc = append_crc_to_data(finish_command)
    send_data(command_with_crc, ser_A)
    return    

def process_request(ser_A, roll, tilt, pan):

    if roll < 0:
        roll = 256 + roll

    if tilt < 0:
        tilt = 256 + tilt

    if pan < 0:
        pan = 256 + pan
    
    command = [92, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 100, 200, 100, 
    0, 0, 10, 138, 90, 166, roll, tilt, pan, 40, 35, 40, 1, 1, 1, 0, 0, 0, 0, 0, 1, 
    0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    command_with_crc = append_crc_to_data(command)
    send_data(command_with_crc, ser_A)
    
    return

def callback_tf(tf_data): ## recent_apriltag_data 관리
    
    # 최근 데이터 저장
    current_time = time_module.time()
    
    # 특정 조건을 만족하는 데이터 필터링, 추가
    if tf_data.transforms and len(tf_data.transforms) > 0: 
        filtered_apriltag_buffer.append((current_time, tf_data.transforms[0]))

    #print("filtered_apriltag_buffer: "+str(len(filtered_apriltag_buffer)))

def manage_apriltag_buffer():
    #filtered_apriltag_buffer 관리#

    current_time = time_module.time()

    # 큐에서 오래된 데이터를 제거
    while filtered_apriltag_buffer and (current_time - filtered_apriltag_buffer[0][0]) > buffer_time:
        filtered_apriltag_buffer.popleft()

    # Debugging
    print("filtered_apriltag_buffer size: ", len(filtered_apriltag_buffer))


# def checkFPS(window_size):
#     # FPS 계산
#     if len(filtered_apriltag_buffer) >= 2:
        
#         time_intervals = [
#             filtered_apriltag_buffer[i + 1][0] - filtered_apriltag_buffer[i][0]
#             for i in range(len(filtered_apriltag_buffer) - 1)
#         ]

#         # 윈도우 내 데이터로 FPS 계산
#         fps = len(time_intervals[-window_size:]) / sum(time_intervals[-window_size:]) ##음수 인덱싱: 끝에서 부터 몇번째
#     else:
#         fps = 0.0

#     print("data fps: " + str(fps))
#     return fps

def create_transformation_matrix(roll, pitch, yaw):

    # Rotation matrix around X-axis (Roll)
    R_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(roll), -np.sin(roll), 0],
        [0, np.sin(roll), np.cos(roll), 0],
        [0, 0, 0, 1]
    ])

    # Rotation matrix around Y-axis (Pitch)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch), 0],
        [0, 1, 0, 0],
        [-np.sin(pitch), 0, np.cos(pitch), 0],
        [0, 0, 0, 1]
    ])

    # Rotation matrix around Z-axis (Yaw)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # Combine the rotations: R = Rz * Rx * RY
    R = np.dot(np.dot(R_z, R_x), R_y)

    # Add translation (assuming no translation here)
    T = np.eye(4)  # Identity matrix for translation
    T[:3, :3] = R[:3, :3]
    return T


def main():
    # 시리얼 포트 설정
    PORT_A = '/dev/gimbal'
    BAUD_RATE = 115200

    ser_A = serial.Serial(PORT_A, BAUD_RATE, timeout=1)

    rospy.init_node('gimbal_test_node', anonymous=True)
    
    rospy.Subscriber("tf", TFMessage, callback_tf)
    angle_pub = rospy.Publisher('gimbal_angle', Float64MultiArray, queue_size=10)
    fps_pub = rospy.Publisher('apriltag_fps', Float64, queue_size=10)
    transformed_pose_pub = rospy.Publisher('transformed_pose', Pose, queue_size=10)


    command_index = 0
    time_var = 0
    period = 1 / 50.0 ## 이걸로 Gimbal control loop hz를 조절할 수 있습니다.

    #reference angle
    ref_roll = 0
    ref_tilt = -90
    ref_pan = 0

    gimbal_roll = 0
    gimbal_tilt = 0
    gimbal_pan = 0

    ##짐벌 control 부분
    try:
        while not rospy.is_shutdown():
            start_time = time_module.time()
            
            # 수신 데이터 처리
            gimbal_roll, gimbal_tilt, gimbal_pan = process_receive(ser_A, angle_pub)
            
            if command_index < len(initial_command):
                # 요청 데이터 전송(Initial Process)
                command_index = process_initial_request(ser_A, command_index)

            elif command_index == len(initial_command):
                print("gimbal 초기화 완료")
                command_index = process_initial_request(ser_A, command_index)
            else:
                # 요청 데이터 전송(Tracking Process)
                manage_apriltag_buffer() # data buffer update
                process_request(ser_A, ref_roll,ref_tilt,ref_pan) #command roll, tilt, pan

                 #apriltag transformation process

                if gimbal_roll is not None:
                    if len(filtered_apriltag_buffer) > 0:
                        if abs(start_time - filtered_apriltag_buffer[-1][0]) < 0.5: #가장 최근 데이터가 0.1s 이내의 데이터면,  
                            #data load
                            latest_data = filtered_apriltag_buffer[-1][1]
                            apriltag_x = latest_data.transform.translation.x
                            apriltag_y = latest_data.transform.translation.y
                            apriltag_z = latest_data.transform.translation.z
                            apriltag_ori_w = latest_data.transform.rotation.w
                            apriltag_ori_x = latest_data.transform.rotation.x
                            apriltag_ori_y = latest_data.transform.rotation.y
                            apriltag_ori_z = latest_data.transform.rotation.z

                            #Transformation process with latest data
                            
                            # Transformation Camera frame --> Gimbal Frame
                            transformation_GC= np.array([
                                [0, 0, 1, 0],
                                [1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 0, 1]
                            ])
                            
                            # Transformation Gimbal frame --> Drone Frame(짐벌 카메라 특성상 드론은 호버링한다고 가정) 생성
                            transformation_DG = create_transformation_matrix(
                                #0 * np.pi / 180, -90* np.pi / 180, 0 * np.pi/180
                                gimbal_roll * np.pi / 180, (gimbal_tilt)*np.pi/180, 0 * np.pi/180
                            )

                            # AprilTag의 위치를 변환 행렬에 곱함
                            apriltag_position = np.array([apriltag_x, apriltag_y, apriltag_z, 1.0])  # 동차 좌표계 사용
                            transformed_position = np.dot(transformation_DG, np.dot(transformation_GC, apriltag_position))

                            # 변환 값을 통한 reference angle 구함

                            if transformed_position[2] == 0:
                                raise ValueError("world_z cannot be zero to avoid division by zero.")
                            else:
                                ref_roll = int(round(np.degrees(np.arctan2(-transformed_position[1], transformed_position[2]))))  # Use arctan2 for robustness

                            if transformed_position[2] == 0:
                                raise ValueError("world_z cannot be zero to avoid division by zero.")
                            else:
                                ref_tilt = int(round(np.degrees(np.arctan2(-transformed_position[2], transformed_position[0]*np.cos(ref_roll*np.pi/180)))))  # Use arctan2 for robustness
                                ref_tilt = max(-130, min(-50, ref_tilt))
                            print("ref roll: "+str(ref_roll) + "," +"ref tilt: "+str(ref_tilt))

                            # ROS Pose 메시지로 변환
                            transformed_msg = Pose()
                            transformed_msg.position.x = transformed_position[0]
                            transformed_msg.position.y = transformed_position[1]
                            transformed_msg.position.z = transformed_position[2]
                            transformed_msg.orientation.x = apriltag_ori_x
                            transformed_msg.orientation.y = apriltag_ori_y
                            transformed_msg.orientation.z = apriltag_ori_z
                            transformed_msg.orientation.w = apriltag_ori_w #apriltag 방향 정보는 일단 그냥 전달

                            # 변환된 좌표 퍼블리시
                            transformed_pose_pub.publish(transformed_msg)

                        else:
                            print("no useful data in buffer")
                            ref_roll, ref_tilt, ref_pan = 0 ,-90, 0
                    else:
                        print("no data in buffer")
                        ref_roll, ref_tilt, ref_pan = 0 ,-90, 0
                else:
                    print("Can't receive Gimbal data")



            # if (start_time - filtered_apriltag_buffer[-1][0]) > 0.1: #가장 최근 데이터가 0.1s 이내의 데이터면,  
            #     latest data = filtered_apriltag_buffer[-1][1]
            # else 
            # # 요청 데이터 전송(default)
            #     command_index = process_request(ser_A, initial_command, default_command, command_index)
             
            # apriltag_fps = checkFPS(window_size)
            # fps_pub.publish(apriltag_fps)

            # 주기 조정
            elapsed_time = time_module.time() - start_time
            sleep_time = max(0, period - elapsed_time)
            print(sleep_time)
            time_module.sleep(sleep_time)

    finally:
        process_finish(ser_A)
        ser_A.close()
        print("프로그램 종료 중...")

if __name__ == '__main__':
    main()