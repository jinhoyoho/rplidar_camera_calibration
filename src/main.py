#!/usr/bin/env python
# -- coding: utf-8 --
import cv2
import rospy
import numpy as np
from cam_cali import CameraCali
from sensor_msgs.msg import PointCloud2
from lidar import LaserToPointCloud
import sensor_msgs.point_cloud2 as pc2


# def Callback(data):
#     global objPoints
#     points = pc2.read_points(data)
#     objPoints = np.array(points)
#     print(objPoints)

if __name__ == '__main__':
    rospy.init_node('main_node', anonymous=True)

    ltp=LaserToPointCloud()
    cam_cali = CameraCali() # intrinsic, extrinsic parameter 구하기
     # Open the video file
    cap = cv2.VideoCapture(4)

    # Loop through the video frames
    while cap.isOpened():
        # Read a frame from the video
        success, frame = cap.read()

        # point_subscriber = rospy.Subscriber("/l2p", PointCloud2, Callback)

        if success:

            # Read points from the cloud
            points = pc2.read_points(ltp.cloud)
            points_list = list(points)
            points_list = [(x, y, z) for x, y, z, _, _ in points_list]

            print("Length of points: ", len(points_list))
            print("First few elements of points: ", points_list[:10])

            # Convert to numpy array and reshape
            objPoints = np.array(points_list, dtype=np.float32).reshape(-1, 3)

            # Display the annotated frame
            img_points, jacobian = cv2.projectPoints(objPoints, cam_cali.rvec, cam_cali.tvec, cam_cali.cameraMatrix, np.array([0,0,0,0],dtype=float))
            print("Shape of img_points: ", img_points.shape)
            print("First few elements of img_points: ", img_points[:5])

            for i in range(len(img_points)):
                # cv2.circle(frame, (int(img_points[i][0]), int(img_points[i][1])), 3, (0,0,255), 1)
                cv2.circle(frame, (int(img_points[i][0][0]), int(img_points[i][0][1])), 3, (0,0,255), 1)

            
            
            cv2.imshow("image", frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # Break the loop if the end of the video is reached
            break

    # Release the video capture object and close the display window
    cap.release()
    cv2.destroyAllWindows()