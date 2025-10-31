#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from swarm_control_msgs.msg import Info
from itertools import combinations

class GraphObserver(Node):
    def __init__(self):
        super().__init__('graph_observer')

        # ---------- Parameters ----------
        self.declare_parameter('num_bots', None)
        self.declare_parameter('delta_radius', 1.5)

        self.num_bots = self.get_parameter('num_bots').get_parameter_value().integer_value
        self.delta_radius = self.get_parameter('delta_radius').get_parameter_value().double_value

        self.bot_list = [f'bot{i+1}' for i in range(self.num_bots)]

        # ---------- Internal state ----------
        self.positions = {b: None for b in self.bot_list}
        self.adjacency = {b: set() for b in self.bot_list}
        self.component_map = {}  # bot_id -> component_id
        self.last_leaders = {}   # component_id -> leader_id

        # ---------- Subscribers ----------
        for bot in self.bot_list:
            self.create_subscription(PoseStamped, f'/{bot}/pose', self.make_pose_cb(bot), 10)

        # ---------- Publishers ----------
        self.info_pubs = {b: self.create_publisher(Info, f'/{b}/info', 10) for b in self.bot_list}

        # ---------- Timer ----------
        self.create_timer(0.2, self.timer_callback)

    # ------------------------------
    # Pose update callback
    # ------------------------------
    def make_pose_cb(self, bot):
        def cb(msg):
            self.positions[bot] = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])
        return cb

    # ------------------------------
    # Graph construction
    # ------------------------------
    def update_adjacency(self):
        for b in self.bot_list:
            self.adjacency[b].clear()

        # Build edges based on distance threshold
        for b1, b2 in combinations(self.bot_list, 2):
            p1, p2 = self.positions[b1], self.positions[b2]
            if p1 is not None and p2 is not None:
                dist = np.linalg.norm(p1 - p2)
                if dist <= self.delta_radius:
                    self.adjacency[b1].add(b2)
                    self.adjacency[b2].add(b1)

    # ------------------------------
    # Component detection (DFS)
    # ------------------------------
    def get_components(self):
        visited = set()
        components = []
        for bot in self.bot_list:
            if bot not in visited:
                stack = [bot]
                component = []
                while stack:
                    curr = stack.pop()
                    if curr not in visited:
                        visited.add(curr)
                        component.append(curr)
                        stack.extend(list(self.adjacency[curr]))
                components.append(component)
        return components

    # ------------------------------
    # Leader election
    # ------------------------------
    def elect_leaders(self, components):
        leaders = {}
        for cid, comp in enumerate(components):
            # Degree of each bot = number of neighbors
            degrees = {b: len(self.adjacency[b]) for b in comp}
            leader = max(degrees, key=degrees.get)
            leaders[cid] = leader
        return leaders

    # ------------------------------
    # Periodic graph update
    # ------------------------------
    def timer_callback(self):
        # 1. Build/update adjacency graph
        self.update_adjacency()

        # 2. Detect components
        components = self.get_components()

        # 3. Elect leaders
        leaders = self.elect_leaders(components)

        # 4. Publish Info messages
        for cid, comp in enumerate(components):
            leader = leaders[cid]
            for bot in comp:
                info = Info()
                info.id = bot
                info.role = "leader" if bot == leader else "agent"
                info.stamp = self.get_clock().now().to_msg()
                info.is_active = True
                info.status_msg = "OK"
                info.component_id = cid
                self.info_pubs[bot].publish(info)

        # 5. Log debug summary
        summary = ", ".join([
            f"Comp{cid}: {comp} (leader={leaders[cid]})"
            for cid, comp in enumerate(components)
        ])
        self.get_logger().debug(f"Graph: {summary}")

def main(args=None):
    rclpy.init(args=args)
    node = GraphObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
