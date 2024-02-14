#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection

class LaserToPointCloud:

    def __init__(self):
        self.laserProj = LaserProjection()
        # self.point_cloud_publisher = rospy.Publisher("/l2p",PointCloud2,queue_size=1)
        self.laser_scan_subscriber = rospy.Subscriber("/scan",LaserScan,self.scanCallback)

    def scanCallback(self,data):
        # Convert laser scan to point cloud
        self.cloud = self.laserProj.projectLaser(data)
        # Publish the point cloud
        # self.point_cloud_publisher.publish(self.cloud)
        print("lidar working")

if __name__ == '__main__':
    rospy.init_node('laser_to_pointcloud', anonymous=True)
    ltp=LaserToPointCloud()
    rospy.spin()