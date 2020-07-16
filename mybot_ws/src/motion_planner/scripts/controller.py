#!/usr/bin/env python
import rospy
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Twist
cmd_vel_pub = None
robot_trans = None
robot_rot = None
grad_vec = np.array([0, 0, 0])
# max value for control
max_w = 0.2
max_v = 0.2


def gradient_callback(data):
    global grad_vec
    grad_quat = data.pose.orientation
    grad_quat = np.array([grad_quat.x, grad_quat.y, grad_quat.z, grad_quat.w])
    r = R.from_quat(grad_quat)
    print(grad_quat)
    grad_rot = r.as_dcm()
    grad_vec = np.dot(grad_rot, np.array([1, 0, 0]))
    # print(grad_vec)
    pass

def calculate_cmd_vel():
    robot_forward = np.array([1, 0, 0])
    robot_forward = np.dot(robot_rot, robot_forward)
    theta = np.arccos(np.dot(grad_vec, robot_forward))
    # clamp theta at 90 degree
    if theta > np.pi/2:
        theta = np.pi/2

    # if cross product positive, grad is on robot left
    cross_prod = np.cross(robot_forward, grad_vec)
    if cross_prod[2] > 0: #left
        w = max_w*np.sin(theta)
        v = max_v*np.cos(theta)
    else:
        w = -max_w*np.sin(theta)
        v = max_v*np.cos(theta)
    cmd_vel = Twist()
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w
    cmd_vel_pub.publish(cmd_vel)
    # print(theta)


def talker():
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist , queue_size=10)



def transform_listener():
    tf_lisener = tf.TransformListener()
    global robot_rot
    global robot_trans
    while not rospy.is_shutdown():
        try:
            (robot_trans,robot_rot) = tf_lisener.lookupTransform('odom', 'chassis', rospy.Time(0))
            r = R.from_quat(robot_rot)
            robot_rot = r.as_dcm()
            calculate_cmd_vel()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

def listener():
    rospy.Subscriber('gradient', PoseStamped, gradient_callback, queue_size=10)
    transform_listener()

if __name__=='__main__':
    print("Starting controller")
    rospy.init_node('robot_controller', anonymous=False)
    talker()
    listener()