#!/usr/bin/env python3
from __future__ import annotations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from typing import Tuple
import math

def get_heading(pose: PoseStamped) -> float:
    orien = pose.pose.orientation
    quat_list = [orien.x, orien.y, orien.z, orien.w]
    heading = euler_from_quaternion(quat_list)
    return heading[2]

def get_orientation(heading: float) -> PoseStamped.orientation:
    orien = PoseStamped().pose.orientation
    quat = quaternion_from_euler(0.0, 0.0, heading)
    (orien.x, orien.y, orien.z, orien.w) = (quat[0], quat[1], quat[2], quat[3])
    return orien

def get_pose_stamped(x: float, y: float, heading: float) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = get_orientation(heading)
    return pose

def get_path(x_coords: list, y_coords: list) -> Path:
    print((x_coords, y_coords))
    count = min(len(x_coords), len(y_coords))
    path = Path()
    for i in range(0, count):
        theta = 0.0
        if i < count - 1:
            theta = math.atan2(y_coords[i + 1] - y_coords[i], x_coords[i + 1] - x_coords[i])
        else:
            theta = math.atan2(y_coords[count - 1] - y_coords[count - 2], x_coords[count - 1] - x_coords[count - 2])
        path.poses.append(get_pose_stamped(x_coords[i], y_coords[i], theta))
    return path

def euclid_distance(p0: Tuple[float, float], p1: Tuple[float, float]) -> float:
    return pow(pow(p1[0] - p0[0], 2.0) + pow(p1[1] - p0[1], 2.0), 0.5)

def get_circle(p_0: Tuple[float, float], p_1: Tuple[float, float], p_2: Tuple[float, float]) -> Tuple[float, float, float]:
    # given points:
    (x_0, y_0) = (p_0[0], p_0[1])
    (x_1, y_1) = (p_1[0], p_1[1])
    (x_2, y_2) = (p_2[0], p_2[1])
    
    # compute coefficients
    A = -(pow(x_0, 2.0)*y_1 - pow(x_0, 2.0)*y_2 - pow(x_1, 2.0)*y_0 + pow(x_1, 2.0)*y_2 + pow(x_2, 2.0)*y_0 - pow(x_2, 2.0)*y_1 + pow(y_0, 2.0)*y_1 - pow(y_0, 2.0)*y_2 - y_0*pow(y_1, 2.0) + y_0*pow(y_2, 2.0) + pow(y_1, 2.0)*y_2 - y_1*pow(y_2, 2.0))/(2*(x_0*y_1 - x_1*y_0 - x_0*y_2 + x_2*y_0 + x_1*y_2 - x_2*y_1))
    B = -(- pow(x_0, 2.0)*x_1 + pow(x_0, 2.0)*x_2 + x_0*pow(x_1, 2.0) - x_0*pow(x_2, 2.0) + x_0*pow(y_1, 2.0) - x_0*pow(y_2, 2.0) - pow(x_1, 2.0)*x_2 + x_1*pow(x_2, 2.0) - x_1*pow(y_0, 2.0) + x_1*pow(y_2, 2.0) + x_2*pow(y_0, 2.0) - x_2*pow(y_1, 2.0))/(2*(x_0*y_1 - x_1*y_0 - x_0*y_2 + x_2*y_0 + x_1*y_2 - x_2*y_1))
    C = (- pow(x_0, 2.0)*x_1*y_2 + pow(x_0, 2.0)*x_2*y_1 + x_0*pow(x_1, 2.0)*y_2 - x_0*pow(x_2, 2.0)*y_1 + x_0*pow(y_1, 2.0)*y_2 - x_0*y_1*pow(y_2, 2.0) - pow(x_1, 2.0)*x_2*y_0 + x_1*pow(x_2, 2.0)*y_0 - x_1*pow(y_0, 2.0)*y_2 + x_1*y_0*pow(y_2, 2.0) + x_2*pow(y_0, 2.0)*y_1 - x_2*y_0*pow(y_1, 2.0))/(x_0*y_1 - x_1*y_0 - x_0*y_2 + x_2*y_0 + x_1*y_2 - x_2*y_1)
    
    # computed circle properties
    x_center = -A
    y_center = -B
    R = pow(pow(A, 2.0) + pow(B, 2.0) - C, 0.5) 
    return (x_center, y_center, R)