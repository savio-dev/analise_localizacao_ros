import rclpy
import yaml
import os
import numpy as np

from rclpy.node import Node
import time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
from ros_analise import motion_model, util
from math import degrees
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from ros_analise.sensor_model import Map, LikelihoodFields, BeamModel
from typing import List

MILLISECONDS = 0.001
ALMOST_ZERO = 1e-15
tempo_inicio = time.time()




class IniciarEpuck(Node):

    def __init__(self):
        super().__init__('iniciar_epuck')
  
       
 

        self._last_used_odom: Pose = None
        self._last_odom: Pose = None
        self._current_pose: Pose = None
        self._last_scan: LaserScan = None

    
        self._map = Map('resource/epuck_world_map.yaml', self.get_logger())
        self._map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
        )

        self._initialize_pose()

        self._tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self._tf_publisher.sendTransform(tf)
        self._publish_map()

   
    def odometry_callback(self, msg: Odometry):
        if not self._updating:
            self._last_odom = msg.pose.pose
           # self.get_logger().info("--- %s seguindos ---" % (time.time() - tempo_inicio))
           

    def scan_callback(self, msg: LaserScan):
        if not self._updating:
            self._last_scan = msg

    def _initialize_pose(self):
        position = Point(x=0.0,
                         y=0.0,
                         z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        self._current_pose = Pose(position=position,
                                  orientation=orientation)
        self._last_used_odom = self._current_pose
        self._last_odom = self._current_pose


    def _publish_map(self):
        map = [-1] * self._map.width * self._map.height
        idx = 0
        for cell in self._map.data:
            map[idx] = int(cell * 100.0)
            idx += 1
        stamp = self.get_clock().now().to_msg()
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.info.resolution = self._map.resolution
        msg.info.width = self._map.width
        msg.info.height = self._map.width
        msg.info.origin.position.x = self._map.origin[0]
        msg.info.origin.position.y = self._map.origin[1]
        msg.data = map
        self._map_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    iniciar_epuck = IniciarEpuck()
    rclpy.spin(iniciar_epuck)
    iniciar_epuck.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
