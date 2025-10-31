#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from swarm_control_msgs.msg import Info, RBroadcast
import numpy as np

class ReferenceNode(Node):
    def __init__(self):
        super().__init__('reference_node')

        # Params
        self.num_bots = self.declare_parameter('num_bots', None).value
        self.time = 0.0
        self.dt = 0.05
        self.leaders = set()

        # Publishers
        self.ref_pub = self.create_publisher(PointStamped, '/reference', 10)
        self.broadcast_pub = self.create_publisher(RBroadcast, '/r_broadcast', 10)

        # Subscribers
        for i in range(1, self.num_bots+1):
            self.create_subscription(Info, f'/bot{i}/info', self.info_callback, 10)

        # Timer
        self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Reference node started")

    def info_callback(self, msg: Info):
        if msg.role == 'leader' and msg.is_active:
            self.leaders.add(msg.id)
        else:
            self.leaders.discard(msg.id)

    def timer_callback(self):
        # Generate r(t)
        t = self.time
        r_vec = np.array([t, 3*np.sin(t)])
        self.time += self.dt

        # Publish /reference
        ref_msg = PointStamped()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.header.frame_id = 'world'
        ref_msg.point.x, ref_msg.point.y = r_vec
        self.ref_pub.publish(ref_msg)

        # If any leaders exist, broadcast reference
        for leader_id in self.leaders:
            bmsg = RBroadcast()
            bmsg.id = leader_id
            bmsg.role = "leader"
            bmsg.stamp = self.get_clock().now().to_msg()
            bmsg.point.x = r_vec[0]
            bmsg.point.y = r_vec[1]
            self.broadcast_pub.publish(bmsg)

        self.get_logger().debug(f"Leaders={list(self.leaders)}, r={r_vec}")

def main(args=None):
    rclpy.init(args=args)
    node = ReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
