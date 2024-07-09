#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import PyLidar3
import time
import math
import tf


port = '/dev/ttyUSB0'
lidar = PyLidar3.YdLidarX4(port)

def publish_lidar_data():
    rospy.init_node('generic_lidar_node', anonymous=True)
    pub = rospy.Publisher('scan', LaserScan, queue_size=10)
    br = tf.TransformBroadcaster()

    if lidar.Connect():
        rospy.loginfo("LIDAR connected successfully")
        gen = lidar.StartScanning()
        rospy.loginfo("LIDAR scanning started")
        
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = (2 * math.pi) / 360
        scan.range_min = 0.12 
        scan.range_max = 3.5   
        
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            scan.header.stamp = rospy.Time.now()
            scan.ranges = []
            scan.intensities = []
            
            data = next(gen)
            for angle in range(360):
                distance = data[angle]
                if distance > 0:  
                    scan.ranges.append(distance / 1000.0)  
                    scan.intensities.append(0)  
                else:
                    scan.ranges.append(float('inf'))
                    scan.intensities.append(0)
            
            pub.publish(scan)

            
            br.sendTransform((0.0, 0.0, 0.1),  
                             tf.transformations.quaternion_from_euler(0, 0, 0),  
                             rospy.Time.now(),
                             "laser",
                             "base_link")
            
            rate.sleep()
        
        lidar.StopScanning()
        lidar.Disconnect()
    else:
        rospy.logerr("LIDAR connection failed")

if __name__ == '__main__':
    try:
        publish_lidar_data()
    except rospy.ROSInterruptException:
        pass
