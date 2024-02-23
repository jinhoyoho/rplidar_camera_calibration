#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection


class LaserToPointCloud:
    def __init__(self):
        # RPLidar uses laser scan, not point cloud
        self.laserProj = LaserProjection()
        self.laser_scan_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.scanCallback)

    def scanCallback(self, data):
        self.cloud = self.laserProj.projectLaser(data)
        print("lidar working")


if __name__ == '__main__':
    rospy.init_node('laser_to_pointcloud', anonymous=True)
    ltp = LaserToPointCloud()
    rospy.spin()
