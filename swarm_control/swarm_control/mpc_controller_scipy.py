#!/usr/bin/env python3
"""
MPC Controller Node for Swarm Control (Scipy Version)

This node implements a Model Predictive Controller that:
1. Subscribes to delta states from kinematic nodes
2. Subscribes to robot odometry 
3. Publishes velocity commands to control robots
4. Uses scipy optimization to minimize tracking error while respecting constraints

Author: Assistant
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
from scipy.optimize import minimize
import time

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # ========== Parameters ==========
        self.declare_parameter('bot_id', None)
        self.declare_parameter('num_bots', None)
        self.declare_parameter('mpc_horizon', 5)
        self.declare_parameter('mpc_dt', 0.1)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('max_linear_acc', 1.0)
        self.declare_parameter('max_angular_acc', 3.0)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('tracking_weight', 10.0)
        self.declare_parameter('control_weight', 1.0)
        
        # Get parameters
        self.bot_id = self.get_parameter('bot_id').get_parameter_value().string_value
        self.num_bots = self.get_parameter('num_bots').get_parameter_value().integer_value
        self.N = self.get_parameter('mpc_horizon').get_parameter_value().integer_value
        self.dt = self.get_parameter('mpc_dt').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.max_linear_acc = self.get_parameter('max_linear_acc').get_parameter_value().double_value
        self.max_angular_acc = self.get_parameter('max_angular_acc').get_parameter_value().double_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.tracking_weight = self.get_parameter('tracking_weight').get_parameter_value().double_value
        self.control_weight = self.get_parameter('control_weight').get_parameter_value().double_value
        
        if not self.bot_id:
            self.get_logger().error("Parameter 'bot_id' not provided! Exiting.")
            raise SystemExit
            
        self.get_logger().info(f"MPC Controller for {self.bot_id} initialized")
        
        # ========== State Variables ==========
        self.current_pose = np.zeros(3)  # [x, y, theta]
        self.current_vel = np.zeros(2)   # [v, omega]
        self.delta_state = np.zeros(2)   # [delta_x, delta_y]
        self.target_delta = np.zeros(2)  # Target delta (usually zero for tracking)
        
        # MPC state history for warm start
        self.prev_solution = None
        self.last_control_time = time.time()
        
        # ========== Publishers ==========
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.bot_id}/cmd_vel', 10)
        
        # ========== Subscribers ==========
        # Own odometry
        self.create_subscription(Odometry, f'/{self.bot_id}/odom', self.odom_callback, 10)
        
        # Own delta state from kinematic node
        self.create_subscription(PointStamped, f'/{self.bot_id}/delta', self.delta_callback, 10)
        
        # ========== Timer ==========
        self.create_timer(1.0/self.control_freq, self.mpc_control_loop)
        
        self.get_logger().info(f"{self.bot_id}: MPC Controller started ✅")
    
    def odom_callback(self, msg: Odometry):
        """Update current robot pose and velocity"""
        # Extract position
        self.current_pose[0] = msg.pose.position.x
        self.current_pose[1] = msg.pose.position.y
        
        # Extract orientation (quaternion to yaw)
        q = msg.pose.orientation
        self.current_pose[2] = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        # Extract velocity
        self.current_vel[0] = msg.twist.twist.linear.x
        self.current_vel[1] = msg.twist.twist.angular.z
    
    def delta_callback(self, msg: PointStamped):
        """Update delta state from kinematic node"""
        self.delta_state[0] = msg.point.x
        self.delta_state[1] = msg.point.y
    
    def robot_dynamics(self, state, control):
        """Robot dynamics for differential drive robot
        
        Args:
            state: [x, y, theta, v, omega]
            control: [a, alpha] (linear and angular acceleration)
            
        Returns:
            state_dot: derivative of state
        """
        x, y, theta, v, omega = state
        a, alpha = control
        
        # Differential drive dynamics
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        theta_dot = omega
        v_dot = a
        omega_dot = alpha
        
        return np.array([x_dot, y_dot, theta_dot, v_dot, omega_dot])
    
    def simulate_trajectory(self, x0, controls):
        """Simulate robot trajectory given initial state and control sequence
        
        Args:
            x0: initial state [x, y, theta, v, omega]
            controls: control sequence [[a0, alpha0], [a1, alpha1], ...]
            
        Returns:
            trajectory: list of states over time horizon
        """
        trajectory = [x0.copy()]
        current_state = x0.copy()
        
        for control in controls:
            # Integrate dynamics using Euler method
            state_dot = self.robot_dynamics(current_state, control)
            current_state += state_dot * self.dt
            
            # Apply velocity constraints
            current_state[3] = np.clip(current_state[3], -self.max_linear_vel, self.max_linear_vel)
            current_state[4] = np.clip(current_state[4], -self.max_angular_vel, self.max_angular_vel)
            
            trajectory.append(current_state.copy())
        
        return trajectory
    
    def mpc_objective(self, control_sequence):
        """Objective function for MPC optimization
        
        Args:
            control_sequence: flattened control sequence [a0, alpha0, a1, alpha1, ...]
            
        Returns:
            cost: scalar cost value
        """
        # Reshape control sequence
        controls = control_sequence.reshape(self.N, 2)
        
        # Current state
        x0 = np.array([
            self.current_pose[0],    # x
            self.current_pose[1],    # y  
            self.current_pose[2],    # theta
            self.current_vel[0],     # v
            self.current_vel[1]      # omega
        ])
        
        # Simulate trajectory
        trajectory = self.simulate_trajectory(x0, controls)
        
        # Compute cost
        cost = 0.0
        
        for k in range(1, len(trajectory)):  # Skip initial state
            state = trajectory[k]
            
            # Tracking cost: minimize delta state
            # Reference position = current position + delta_state
            ref_x = self.current_pose[0] + self.delta_state[0]
            ref_y = self.current_pose[1] + self.delta_state[1]
            
            tracking_error = (state[0] - ref_x)**2 + (state[1] - ref_y)**2
            cost += self.tracking_weight * tracking_error
            
            # Control cost: minimize acceleration
            control_cost = controls[k-1][0]**2 + controls[k-1][1]**2
            cost += self.control_weight * control_cost
            
            # Terminal cost (penalize final tracking error more)
            if k == len(trajectory) - 1:
                cost += 5.0 * self.tracking_weight * tracking_error
        
        return cost
    
    def mpc_constraints(self, control_sequence):
        """Constraint function for MPC optimization
        
        Args:
            control_sequence: flattened control sequence
            
        Returns:
            constraint_values: constraint violations (should be <= 0)
        """
        controls = control_sequence.reshape(self.N, 2)
        
        # Current state
        x0 = np.array([
            self.current_pose[0], self.current_pose[1], self.current_pose[2],
            self.current_vel[0], self.current_vel[1]
        ])
        
        # Simulate trajectory
        trajectory = self.simulate_trajectory(x0, controls)
        
        constraints = []
        
        for k in range(1, len(trajectory)):
            state = trajectory[k]
            
            # Velocity constraints
            constraints.append(state[3] - self.max_linear_vel)   # v <= v_max
            constraints.append(-state[3] - self.max_linear_vel)    # -v <= v_max
            constraints.append(state[4] - self.max_angular_vel)    # omega <= omega_max
            constraints.append(-state[4] - self.max_angular_vel)  # -omega <= omega_max
            
            # Acceleration constraints
            constraints.append(controls[k-1][0] - self.max_linear_acc)    # a <= a_max
            constraints.append(-controls[k-1][0] - self.max_linear_acc) # -a <= a_max
            constraints.append(controls[k-1][1] - self.max_angular_acc)   # alpha <= alpha_max
            constraints.append(-controls[k-1][1] - self.max_angular_acc) # -alpha <= alpha_max
        
        return np.array(constraints)
    
    def solve_mpc(self):
        """Solve the MPC optimization problem using scipy"""
        try:
            # Initial guess (warm start)
            if self.prev_solution is not None:
                x0 = self.prev_solution[1:].flatten()  # Shift previous solution
                x0 = np.append(x0, self.prev_solution[-1])  # Repeat last control
            else:
                x0 = np.zeros(2 * self.N)
            
            # Bounds for control variables
            bounds = []
            for _ in range(self.N):
                bounds.append((-self.max_linear_acc, self.max_linear_acc))   # a bounds
                bounds.append((-self.max_angular_acc, self.max_angular_acc)) # alpha bounds
            
            # Solve optimization problem
            result = minimize(
                fun=self.mpc_objective,
                x0=x0,
                method='SLSQP',
                bounds=bounds,
                constraints={'type': 'ineq', 'fun': lambda x: -self.mpc_constraints(x)},
                options={'maxiter': 100, 'ftol': 1e-6}
            )
            
            if result.success:
                # Extract solution
                u_opt = result.x.reshape(self.N, 2)
                return u_opt
            else:
                self.get_logger().warn(f"{self.bot_id}: MPC optimization failed: {result.message}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"{self.bot_id}: MPC solve error: {str(e)}")
            return None
    
    def mpc_control_loop(self):
        """Main MPC control loop"""
        try:
            # Solve MPC problem
            optimal_control = self.solve_mpc()
            
            if optimal_control is not None:
                # Apply first control action
                self.apply_control(optimal_control[0])
                
                # Store solution for warm start
                self.prev_solution = optimal_control
            else:
                self.get_logger().warn(f"{self.bot_id}: MPC optimization failed, using zero control")
                self.apply_control(np.zeros(2))
                
        except Exception as e:
            self.get_logger().error(f"{self.bot_id}: MPC control error: {str(e)}")
            self.apply_control(np.zeros(2))
    
    def apply_control(self, control):
        """Apply control command to robot"""
        # control = [a, alpha] (acceleration)
        # Convert to velocity commands for differential drive
        
        cmd_vel = Twist()
        
        # Simple conversion: use acceleration as velocity command
        # In practice, you might want to integrate this properly
        cmd_vel.linear.x = float(control[0])
        cmd_vel.angular.z = float(control[1])
        
        # Apply velocity limits
        cmd_vel.linear.x = np.clip(cmd_vel.linear.x, -self.max_linear_vel, self.max_linear_vel)
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, -self.max_angular_vel, self.max_angular_vel)
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Debug logging
        self.get_logger().debug(
            f"{self.bot_id}: cmd_vel=[{cmd_vel.linear.x:.3f}, {cmd_vel.angular.z:.3f}], "
            f"delta=[{self.delta_state[0]:.3f}, {self.delta_state[1]:.3f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


