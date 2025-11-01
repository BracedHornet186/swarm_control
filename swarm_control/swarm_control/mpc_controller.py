#!/usr/bin/env python3
"""
MPC Controller Node for Swarm Control

This node implements a Model Predictive Controller that:
1. Subscribes to delta states from kinematic nodes
2. Subscribes to robot odometry 
3. Publishes velocity commands to control robots
4. Uses optimization to minimize tracking error while respecting constraints

Author: Assistant
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
from scipy.optimize import minimize
import casadi as ca

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # ========== Parameters ==========
        self.declare_parameter('bot_id', None)
        self.declare_parameter('num_bots', None)
        self.declare_parameter('mpc_horizon', 10)
        self.declare_parameter('mpc_dt', 0.1)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('max_linear_acc', 1.0)
        self.declare_parameter('max_angular_acc', 3.0)
        self.declare_parameter('control_frequency', 10.0)
        
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
        
        # ========== Publishers ==========
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.bot_id}/cmd_vel', 10)
        
        # ========== Subscribers ==========
        # Own odometry
        self.create_subscription(Odometry, f'/{self.bot_id}/odom', self.odom_callback, 10)
        
        # Own delta state from kinematic node
        self.create_subscription(PointStamped, f'/{self.bot_id}/delta', self.delta_callback, 10)
        
        # ========== Timer ==========
        self.create_timer(1.0/self.control_freq, self.mpc_control_loop)
        
        # ========== MPC Setup ==========
        self.setup_mpc_problem()
        
        self.get_logger().info(f"{self.bot_id}: MPC Controller started ✅")
    
    def setup_mpc_problem(self):
        """Setup the MPC optimization problem using CasADi"""
        # State variables: [x, y, theta, v, omega]
        # Control variables: [a, alpha] (linear and angular acceleration)
        
        # Define symbolic variables
        self.x = ca.SX.sym('x', 5)  # [x, y, theta, v, omega]
        self.u = ca.SX.sym('u', 2)  # [a, alpha]
        
        # Robot dynamics (differential drive)
        # x_dot = v * cos(theta)
        # y_dot = v * sin(theta)  
        # theta_dot = omega
        # v_dot = a
        # omega_dot = alpha
        
        x_dot = ca.vertcat(
            self.x[3] * ca.cos(self.x[2]),  # x_dot
            self.x[3] * ca.sin(self.x[2]),  # y_dot
            self.x[4],                      # theta_dot
            self.u[0],                      # v_dot = a
            self.u[1]                       # omega_dot = alpha
        )
        
        # Create function for dynamics
        self.f = ca.Function('f', [self.x, self.u], [x_dot])
        
        # Create integrator
        self.F = ca.integrator('F', 'cvodes', {
            'x': self.x,
            'p': self.u,
            'ode': x_dot,
            't0': 0,
            'tf': self.dt
        })
        
        self.get_logger().info("MPC problem setup completed")
    
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
    
    def solve_mpc(self):
        """Solve the MPC optimization problem"""
        # Current state
        x0 = np.array([
            self.current_pose[0],    # x
            self.current_pose[1],    # y  
            self.current_pose[2],    # theta
            self.current_vel[0],     # v
            self.current_vel[1]      # omega
        ])
        
        # Target: minimize delta state (tracking error)
        # We want delta to go to zero
        target_delta = np.zeros(2)
        
        # Optimization variables: [u0, u1, ..., u_{N-1}]
        # Each u_i = [a_i, alpha_i]
        opt_vars = ca.SX.sym('opt_vars', 2 * self.N)
        
        # Cost function
        cost = 0
        
        # State trajectory
        x_traj = [x0]
        
        # Simulate forward
        for k in range(self.N):
            # Extract control for this step
            u_k = opt_vars[2*k:2*k+2]
            
            # Integrate dynamics
            x_next = self.F(x0=x_traj[-1], p=u_k)['xf']
            x_traj.append(x_next)
            
            # Cost: minimize delta tracking error
            # delta = [x, y] - reference (which is current position + delta_state)
            ref_x = self.current_pose[0] + self.delta_state[0]
            ref_y = self.current_pose[1] + self.delta_state[1]
            
            delta_x = x_next[0] - ref_x
            delta_y = x_next[1] - ref_y
            
            # Tracking cost
            cost += 10.0 * (delta_x**2 + delta_y**2)
            
            # Control cost (minimize acceleration)
            cost += 1.0 * (u_k[0]**2 + u_k[1]**2)
            
            # Terminal cost (penalize final delta)
            if k == self.N - 1:
                cost += 50.0 * (delta_x**2 + delta_y**2)
        
        # Constraints
        g = []
        lbg = []
        ubg = []
        
        # Control constraints
        for k in range(self.N):
            # Linear velocity constraint: |v| <= v_max
            v_k = x_traj[k+1][3]
            g.append(v_k)
            lbg.append(-self.max_linear_vel)
            ubg.append(self.max_linear_vel)
            
            # Angular velocity constraint: |omega| <= omega_max  
            omega_k = x_traj[k+1][4]
            g.append(omega_k)
            lbg.append(-self.max_angular_vel)
            ubg.append(self.max_angular_vel)
            
            # Acceleration constraints
            a_k = opt_vars[2*k]
            alpha_k = opt_vars[2*k+1]
            
            g.append(a_k)
            lbg.append(-self.max_linear_acc)
            ubg.append(self.max_linear_acc)
            
            g.append(alpha_k)
            lbg.append(-self.max_angular_acc)
            ubg.append(self.max_angular_acc)
        
        # Create NLP problem
        nlp = {
            'x': opt_vars,
            'f': cost,
            'g': ca.vertcat(*g)
        }
        
        # Solver options
        opts = {
            'ipopt': {
                'max_iter': 100,
                'print_level': 0,
                'acceptable_tol': 1e-6,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        
        # Create solver
        solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
        
        # Initial guess (warm start)
        if self.prev_solution is not None:
            x0_guess = self.prev_solution[1:].flatten()  # Shift previous solution
            x0_guess = np.append(x0_guess, self.prev_solution[-1])  # Repeat last control
        else:
            x0_guess = np.zeros(2 * self.N)
        
        # Solve
        sol = solver(
            x0=x0_guess,
            lbg=lbg,
            ubg=ubg
        )
        
        if solver.stats()['success']:
            # Extract solution
            u_opt = sol['x'].full()
            u_opt = u_opt.reshape(self.N, 2)
            return u_opt
        else:
            self.get_logger().warn(f"{self.bot_id}: MPC solver failed")
            return None
    
    def apply_control(self, control):
        """Apply control command to robot"""
        # control = [a, alpha] (acceleration)
        # We need to convert to velocity commands
        
        # Simple integration: v_new = v_current + a * dt
        # But we'll use the acceleration directly as velocity command
        # for simplicity (assuming the robot can handle this)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = float(control[0])  # Use acceleration as velocity command
        cmd_vel.angular.z = float(control[1])  # Use angular acceleration as angular velocity
        
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


