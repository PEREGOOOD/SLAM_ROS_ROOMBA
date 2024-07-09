#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from pynput import keyboard
import pycreate2
import tf
from math import sin, cos


rospy.init_node('roomba_control', anonymous=True)

port = '/dev/ttyUSB1'  
baud = 115200

bot = pycreate2.Create2(port, baud)
bot.start()
bot.safe()


x = 0.0
y = 0.0
th = 0.0
vx = 0.0
vth = 0.0
last_time = rospy.Time.now()


odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

def move_roomba(linear, angular):
    global vx, vth
    """Convert linear and angular velocities to Roomba's drive command."""
    left_wheel_velocity = int((linear - angular) * 500)
    right_wheel_velocity = int((linear + angular) * 500)
    bot.drive_direct(left_wheel_velocity, right_wheel_velocity)
    rospy.loginfo(f"Moving Roomba: linear={linear}, angular={angular}, left_wheel={left_wheel_velocity}, right_wheel={right_wheel_velocity}")
    vx = linear
    vth = angular

def publish_odometry(event):
    global x, y, th, last_time, vx, vth
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

  
    wheel_base = 0.235  
    delta_x = vx * cos(th) * dt
    delta_y = vx * sin(th) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    
    odom_broadcaster.sendTransform(
        (x, y, 0.0),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

    
    odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                            0, 0.1, 0, 0, 0, 0,
                            0, 0, 0.1, 0, 0, 0,
                            0, 0, 0, 0.1, 0, 0,
                            0, 0, 0, 0, 0.1, 0,
                            0, 0, 0, 0, 0, 0.1]
    odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                             0, 0.1, 0, 0, 0, 0,
                             0, 0, 0.1, 0, 0, 0,
                             0, 0, 0, 0.1, 0, 0,
                             0, 0, 0, 0, 0.1, 0,
                             0, 0, 0, 0, 0, 0.1]

    odom_pub.publish(odom)
    last_time = current_time


def on_press(key):
    linear = 0.0
    angular = 0.0
    try:
        if key.char == 'w':
            linear = 0.2
        elif key.char == 's':
            linear = -0.2
        elif key.char == 'd':
            angular = 1.0
        elif key.char == 'a':
            angular = -1.0
        rospy.loginfo(f"Key pressed: {key.char}")
    except AttributeError:
        rospy.loginfo(f"Special key pressed: {key}")
    move_roomba(linear, angular)

def on_release(key):
    move_roomba(0.0, 0.0)
    rospy.loginfo(f"Key released: {key}")
    if key == keyboard.Key.esc:
        # Stop listener
        return False


rospy.Timer(rospy.Duration(0.1), publish_odometry)


with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    rospy.loginfo("Starting keyboard listener")
    listener.join()


bot.drive_stop()
