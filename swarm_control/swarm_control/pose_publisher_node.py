#!/usr/bin/env python3
"""
Gazebo Transport → ROS 2 Pose Relay Node
---------------------------------------
Subscribes to Gazebo topic /world/default/pose/info (gz.msgs.Pose_V)
and republishes each model's pose to /<model_name>/pose as geometry_msgs/PoseStamped.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import gz.transport14 as gz
from gz.msgs11 import pose_v_pb2
from threading import Thread
import time


class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        
        self.declare_parameter('num_bots', None)
        self.num_bots = self.get_parameter('num_bots').get_parameter_value().integer_value

        # Dictionary of ROS 2 publishers (one per model)
        self.pose_publishers = {f'bot{i+1}': self.create_publisher(PoseStamped, f'/{f"bot{i+1}/pose"}', 10) for i in range(self.num_bots)}

        # Create a background Gazebo subscriber thread
        self.gz_node = gz.Node()
        self.gz_node.subscribe(pose_v_pb2.Pose_V, '/world/default/pose/info', self.gz_callback)

        # Spin a Gazebo transport thread (non-blocking)
        self.gz_thread = Thread(target=self._gz_spin, daemon=True)
        self.gz_thread.start()

        self.get_logger().info("GazeboPoseRelay started. Listening on /world/default/pose/info ...")

    # -------------------------------------------------
    # Gazebo transport callback
    # -------------------------------------------------
    def gz_callback(self, msg_bytes):
        # Deserialize Gazebo message

        now = self.get_clock().now().to_msg()

        for p in msg_bytes.pose:
            model_name = p.name
            if model_name == '':
                continue  # skip unnamed entries

            # Lazy-create ROS publisher if not already present
            if model_name in self.pose_publishers.keys():

                # Build PoseStamped message
                ros_msg = PoseStamped()
                ros_msg.header.stamp = now
                ros_msg.header.frame_id = 'world'

                ros_msg.pose.position.x = p.position.x
                ros_msg.pose.position.y = p.position.y
                ros_msg.pose.position.z = p.position.z
                ros_msg.pose.orientation.x = p.orientation.x
                ros_msg.pose.orientation.y = p.orientation.y
                ros_msg.pose.orientation.z = p.orientation.z
                ros_msg.pose.orientation.w = p.orientation.w

                # Publish on ROS 2
                self.pose_publishers[model_name].publish(ros_msg)

    # -------------------------------------------------
    # Gazebo node spin thread
    # -------------------------------------------------
    def _gz_spin(self):
        """Continuously process Gazebo messages in a background thread."""
        while rclpy.ok():
            time.sleep(0.001)  # prevents busy-wait


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
