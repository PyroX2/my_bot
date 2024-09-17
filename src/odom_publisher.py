from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self._subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.callback, qos)
