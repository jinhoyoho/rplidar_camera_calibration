#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

CV_BRIDGE = CvBridge()

if __name__ == '__main__':
    pub = rospy.Publisher('image_topic', Image, queue_size=1)
    rospy.init_node('image_publisher')
    # Open the video file
    cap = cv2.VideoCapture(4)

    # Loop through the video frames
    while cap.isOpened():
        # Read a frame from the video
        success, frame = cap.read()

        if success:
            # Display the annotated frame
            cv2.imshow("image", frame)
            msg = CV_BRIDGE.cv2_to_imgmsg(frame, 'bgr8')

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # Break the loop if the end of the video is reached
            break

    # Release the video capture object and close the display window
    cap.release()
    cv2.destroyAllWindows()