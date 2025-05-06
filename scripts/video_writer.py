#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

# 비디오 저장 객체 생성
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 'mp4v'로 바꾸면 mp4도 가능
fps = 20.0  # 초당 프레임 수 (데이터에 맞게 설정)
frame_size = (800, 600)  # 이미지 크기 (width, height)

# .avi 또는 .mp4 파일 이름
PATH = '/home/bwh/Dropbox/px4/QGroundControl/Logs/pixhawk4/'
out = cv2.VideoWriter(f'{PATH}/gimbal_camera.avi', fourcc, fps, frame_size)

def callback(msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if image_np is not None:
        # 크기 맞추기 (필요 시)
        resized = cv2.resize(image_np, frame_size)
        out.write(resized)  # 비디오에 프레임 추가

def listener():
    rospy.init_node('video_writer_node', anonymous=True)
    rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, callback)

    rospy.loginfo(f"Video Writer Node Started. Saving to {PATH}.")

    rospy.spin()
    out.release()

if __name__ == '__main__':
    listener()
