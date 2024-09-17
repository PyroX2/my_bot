import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint
from geometry_msgs.msg import Twist
import numpy as np


class TorqueThrustPublihser(Node):
    def __init__(self):
        super().__init__('torque_thrust_publihser')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._subscription = self.create_subscription(
            Twist, '/cmd_vel', self.callback, 10)
        self.thrust_publisher = self.create_publisher(
            VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile)
        self.torque_publisher = self.create_publisher(
            VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', qos_profile)

        self._subscription

    def callback(self, msg):
        px4_thrust_setpoint = VehicleThrustSetpoint()
        px4_torque_setpoint = VehicleTorqueSetpoint()

        linear = [msg.linear.x, msg.linear.y, msg.linear.z]
        angular = [msg.angular.x, msg.angular.y, -msg.angular.z]
        px4_thrust_setpoint.xyz = linear
        px4_torque_setpoint.xyz = angular

        self.thrust_publisher.publish(px4_thrust_setpoint)
        self.torque_publisher.publish(px4_torque_setpoint)
        print('Published')


def main():
    rclpy.init()
    node = TorqueThrustPublihser()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
