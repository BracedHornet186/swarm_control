#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import numpy as np
from swarm_control_msgs.msg import Info, RBroadcast

# ===========================================
# Parameters for the kinematic model
# ===========================================
T_S = 0.05           # Sampling period [s]
M = -np.eye(2)       # Coupling gain
NU = 1.0             # Exponent in denominator
EPS = 1e-2           # Small epsilon to avoid division by zero
DELTA_RADIUS = 3.0   # Communication radius (m)

class KinematicNode(Node):
    def __init__(self):
        super().__init__('kinematic_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('bot_id', None)
        self.declare_parameter('num_bots', None)
        self.declare_parameter('delta_radius', DELTA_RADIUS)
        self.declare_parameter('role', 'agent')

        self.bot_id = self.get_parameter('bot_id').get_parameter_value().string_value
        self.num_bots = self.get_parameter('num_bots').get_parameter_value().integer_value
        self.delta_radius = self.get_parameter('delta_radius').get_parameter_value().double_value
        self.role = self.get_parameter('role').get_parameter_value().string_value

        if not self.bot_id:
            self.get_logger().error("Parameter 'bot_id' not provided! Exiting.")
            raise SystemExit

        # Dynamically generate the list of all bots
        self.agent_list = [f'bot{i+1}' for i in range(self.num_bots)]
        if self.bot_id in self.agent_list:
            self.agent_list.remove(self.bot_id)

        self.get_logger().info(f"{self.bot_id}: tracking {len(self.agent_list)} agents -> {self.agent_list}")        

        # ---------------- State Variables ----------------
        self.pose_dict = {b: None for b in self.agent_list}
        self.my_pose = None
        self.component_dict = {b: None for b in self.agent_list}
        self.my_cid = None
        self.delta = np.zeros(2)
        self.delta_dict = {b: np.zeros(2) for b in self.agent_list}
        self.r_vec = np.zeros(2)   # reference point from r_broadcast

        # ---------------- Publishers ----------------
        self.delta_pub = self.create_publisher(PointStamped, f'/{self.bot_id}/delta', 10)

        # ---------------- Subscribers ----------------
        # Own Pose and Info
        self.create_subscription(PoseStamped, f'/{self.bot_id}/pose', self.pose_callback, 10)
        self.create_subscription(Info, f'/{self.bot_id}/info', self.info_callback, 10)

        # Other bots' pose and delta
        for bot in self.agent_list:
            self.create_subscription(PoseStamped, f'/{bot}/pose', self.make_pose_cb(bot), 10)
            self.create_subscription(Info, f'/{bot}/info', self.make_info_cb(bot), 10)
            self.create_subscription(PointStamped, f'/{bot}/delta', self.make_delta_cb(bot), 10)

        # Reference broadcast
        self.create_subscription(RBroadcast, '/r_broadcast', self.r_cb, 10)

        # ---------------- Timer ----------------
        self.create_timer(T_S, self.timer_callback)
        self.get_logger().info(f"{self.bot_id}: Kinematic node started")

    # ===========================================
    # Callbacks
    # ===========================================
    def pose_callback(self, msg: PoseStamped):
        self.my_pose = np.array([msg.pose.position.x,
                                 msg.pose.position.y])
        
    def info_callback(self, msg: Info):
        self.my_cid = msg.component_id

    def make_pose_cb(self, bot):
        """Closure for storing pose of other bots."""
        def cb(msg: PoseStamped):
            self.pose_dict[bot] = np.array([msg.pose.position.x,
                                            msg.pose.position.y])
        return cb

    def make_info_cb(self, bot):
        """Closure for storing info of other bots."""
        def cb(msg: Info):
            self.component_dict[bot] = msg.component_id
        return cb
    
    def make_delta_cb(self, bot):
        """Closure for storing delta of other bots."""
        def cb(msg):
            self.delta_dict[bot] = np.array([msg.point.x, msg.point.y])
        return cb

    def r_cb(self, msg: RBroadcast):
        """Callback for receiving reference trajectory from leader."""
        self.r_vec = np.array([msg.point.x, msg.point.y])
        self.get_logger().debug(
            f"{self.bot_id}: Received r_broadcast from {msg.id}: r={self.r_vec}"
        )

    # def compute_neighbors(self):
    #     """Return list of neighbors within delta_radius based on poseetry."""
    #     neighbors = []
    #     for bot, pose in self.pose_dict.items():
    #         if pose is not None:
    #             dist = np.linalg.norm(self.my_pose - pose)
    #             if dist <= self.delta_radius:
    #                 neighbors.append(bot)
    #     return neighbors

    # ===========================================
    # Main periodic update
    # ===========================================
    def timer_callback(self):
        # Skip until both pose and component ID are known
        if self.my_pose is None or self.my_cid is None:
            self.get_logger().warning(f"{self.bot_id}: Waiting for pose/component ID...")
            return
        
        # 1. Compute Δ_i = z_i - r(t)
        self.delta = self.my_pose - self.r_vec

        # 2. Find component members
        component_members = [b for b, cid in self.component_dict.items() if cid == self.my_cid]
        n_cc = max(1, len(component_members) + 1)

        # 3. Compute coupling term
        coupling = np.zeros(2)
        for nb in component_members:
            diff = self.delta - self.delta_dict[nb]
            denom = np.linalg.norm(diff)**NU + EPS
            coupling += diff / denom

        # 4. Δ̇ update and Euler integration
        d_delta = -self.delta - (M / n_cc) * coupling
        self.delta += T_S * d_delta

        # 5. Publish updated delta
        delta_msg = PointStamped()
        delta_msg.header.stamp = self.get_clock().now().to_msg()
        delta_msg.header.frame_id = 'world'
        delta_msg.point.x, delta_msg.point.y = self.delta
        self.delta_pub.publish(delta_msg)

        # Debug log
        self.get_logger().debug(
            f"{self.bot_id}: Δ={self.delta.round(3)}, comp={self.my_cid}, members={component_members}"
    )

# ===========================================
# Node entry point
# ===========================================
def main(args=None):
    rclpy.init(args=args)
    node = KinematicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()