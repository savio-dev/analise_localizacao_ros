import numpy as np

from geometry_msgs.msg import Quaternion
from math import atan2, asin, pi
from ros_analise.sensor_model import Map


def yaw_from_quaternion(orientation: Quaternion):
    _, _, yaw = euler_from_quaternion(orientation.x,
                                      orientation.y,
                                      orientation.z,
                                      orientation.w)
    return yaw


def location_to_map_location(x: float, y: float, map: Map) -> tuple:
    map_x = int((x + map.origin[0]) / map.resolution)
    map_y = int((y + map.origin[1]) / map.resolution)
    return (map_x, map_y)


def map_location_to_location(map_x: float, map_y: float, map: Map) -> tuple:
    x = float(map_x * map.resolution + map.origin[0])
    y = float(map_y * map.resolution + map.origin[1])
    return (x, y)


def location_in_map_meter(x: float, y: float, map: Map) -> tuple:
    map_x = x - map.origin[0]
    map_y = y - map.origin[1]
    return (map_x, map_y)


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def euler_to_quaternion(yaw, pitch, roll) -> Quaternion:
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def angle_diff(angle1: float, angle2: float) -> float:
    d1 = angle1 - angle2
    d2 = 2 * pi - abs(d1)
    if d1 > 0.0:
        d2 *= -1.0

    if abs(d1) < abs(d2):
        return d1
    else:
        return d2
