#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('siyi_rtsp_publisher')
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # GStreamer pipeline for RTSP
    cap = cv2.VideoCapture(
        "rtspsrc location=rtsp://192.168.144.25:8554/live latency=100 ! decodebin ! videoconvert ! appsink",
        cv2.CAP_GSTREAMER
    )

    if not cap.isOpened():
        rospy.logerr("Failed to open RTSP stream")
        return

    rate = rospy.Rate(30)  # 30 FPS

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("No frame received")
            continue

        msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    main()
