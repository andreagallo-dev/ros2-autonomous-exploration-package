#!/usr/bin/env python3

# ==========================================
# IMPORTS
# ==========================================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import math
import random
import time
import os

# ==========================================
# NODE CLASS DEFINITION
# ==========================================
class UltimateExplorer(Node):
    def __init__(self):
        super().__init__('ultimate_explorer')
        
        # --- CONFIGURATION PARAMETERS ---
        self.frontier_threshold = 25   # If valid frontier pixels drop below this, exploration is complete
        self.MAX_NAV_TIME = 35.0       # Max allowed time (seconds) to reach a goal before triggering timeout
        
        # --- ROS 2 COMPONENTS ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- INTERNAL STATE VARIABLES ---
        self.latest_map = None         
        self.blacklist_zones = []      # List of (x,y) unreachable coordinates to ignore
        
        self.is_navigating = False     
        self.nav_start_time = None     
        self.current_goal_handle = None
        self.current_target = None     
        
        self.home_pose = None          
        self.returning_home = False    

        # Wait for Nav2 to be fully online before starting
        self.get_logger().info("Waiting for Nav2 Action Server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 Ready! Starting autonomous exploration.")
        
        # Main logic loop (1 Hz)
        self.timer = self.create_timer(1.0, self.planning_loop)

    # ==========================================
    # CALLBACKS & UTILITIES
    # ==========================================
    def map_callback(self, msg):
        """Updates the latest map received from SLAM"""
        self.latest_map = msg

    def get_robot_pose(self):
        """Queries TF2 for the current robot pose relative to the map frame"""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Quaternion to Euler (Yaw) conversion
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except Exception:
            # Suppress errors during initial startup if TF is not yet available
            return None, None, None

    # ==========================================
    # MAIN LOGIC LOOP (Executed every 1 sec)
    # ==========================================
    def planning_loop(self):
        robot_x, robot_y, robot_yaw = self.get_robot_pose()
        if robot_x is None or self.latest_map is None:
            return

        # 0. SAVE HOME POSE (Executed only once at startup)
        if self.home_pose is None:
            self.home_pose = (robot_x, robot_y, robot_yaw)
            self.get_logger().info(f"🏠 HOME pose registered at: X={robot_x:.2f}, Y={robot_y:.2f}")

        # 1. NAVIGATION TIMEOUT HANDLER
        if self.is_navigating:
            if time.time() - self.nav_start_time > self.MAX_NAV_TIME:
                self.get_logger().warn("⚠️ TIMEOUT! Robot appears stuck. Canceling goal.")
                
                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
                
                # If stuck while homing, save map at current location
                if self.returning_home:
                    self.get_logger().warn("Cannot reach Home perfectly. Saving map here.")
                    self.save_map_and_exit()
                else:
                    # Blacklist the unreachable target to prevent repetitive failures
                    self.blacklist_zones.append(self.current_target)
                    self.is_navigating = False 
            return 

        # Skip frontier generation if the robot is already on its way home
        if self.returning_home:
            return

        self.get_logger().info("Analyzing map for new frontiers...")

        # --- MAP DATA EXTRACTION ---
        width = self.latest_map.info.width          
        height = self.latest_map.info.height        
        resolution = self.latest_map.info.resolution 
        origin_x = self.latest_map.info.origin.position.x 
        origin_y = self.latest_map.info.origin.position.y 
        
        # Convert 1D ROS map array into 2D NumPy array
        # Values: 0 = Free, 100 = Occupied, -1 = Unknown
        map_data = np.array(self.latest_map.data).reshape((height, width))
        
        free_space = (map_data == 0)
        unknown_space = (map_data == -1)
        
        # --- FRONTIER EXTRACTION VIA NUMPY SHIFTING ---
        # Shift the 'unknown_space' matrix in all 4 directions
        shift_up = np.roll(unknown_space, 1, axis=0)
        shift_down = np.roll(unknown_space, -1, axis=0)
        shift_left = np.roll(unknown_space, 1, axis=1)
        shift_right = np.roll(unknown_space, -1, axis=1)
        
        # A pixel is a frontier if it is free space AND at least one neighbor is unknown
        is_frontier = free_space & (shift_up | shift_down | shift_left | shift_right)
        fy, fx = np.where(is_frontier)
        
        valid_frontiers = []
        
        # --- FRONTIER FILTERING ---
        for i in range(len(fx)):
            wx = (fx[i] * resolution) + origin_x
            wy = (fy[i] * resolution) + origin_y
            
            dist = math.hypot(wx - robot_x, wy - robot_y)
            
            # Filter 1: Ignore points too close to the robot (< 1.0m) to avoid micro-movements
            if dist < 1.0:
                continue
                
            # Filter 2: Check if the point falls within the radius (0.6m) of a blacklisted zone
            in_blacklist = any(math.hypot(wx - bx, wy - by) < 0.6 for bx, by in self.blacklist_zones)
            if not in_blacklist:
                valid_frontiers.append((wx, wy, dist))

        # 2. AUTO-COMPLETION & HOMING CONDITION
        if len(valid_frontiers) < self.frontier_threshold:
            self.get_logger().info("✅ EXPLORATION COMPLETED! Returning to Home Base 🔋...")
            self.returning_home = True
            
            # Increase timeout since crossing the entire house takes more time
            self.MAX_NAV_TIME = 90.0 
            self.send_goal(self.home_pose[0], self.home_pose[1], self.home_pose[2])
            return

        # --- TARGET SELECTION (Long-Stride Heuristic) ---
        # Sort valid frontiers by distance
        valid_frontiers.sort(key=lambda f: f[2])
        
        # Keep only the top 30% furthest frontiers
        top_far_frontiers = valid_frontiers[int(len(valid_frontiers) * 0.7):]
        if not top_far_frontiers:
            top_far_frontiers = valid_frontiers
            
        # Stochastically select a distant target to avoid deterministic loops
        best_goal = random.choice(top_far_frontiers)
        
        # Send goal to Nav2 (Yaw is set to 0.0 as orientation towards walls is irrelevant)
        self.send_goal(best_goal[0], best_goal[1], 0.0)

    # ==========================================
    # NAV2 COMMUNICATION (ACTION CLIENT)
    # ==========================================
    def send_goal(self, x, y, yaw):
        self.is_navigating = True
        self.current_target = (x, y)
        self.nav_start_time = time.time() 
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if self.returning_home:
            self.get_logger().info(f"Navigating HOME: X={x:.2f}, Y={y:.2f}")
        else:
            self.get_logger().info(f"Long-stride towards frontier: X={x:.2f}, Y={y:.2f}")
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        
        # Check if the Nav2 planner rejects the goal (e.g., target inside a wall)
        if not self.current_goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by planner. Adding to Blacklist.')
            if self.returning_home:
                self.save_map_and_exit()
            else:
                self.blacklist_zones.append(self.current_target)
                self.is_navigating = False
            return

        # Wait for the physical action to complete
        self._get_result_future = self.current_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.is_navigating = False 
        
        if status == 4: # SUCCEEDED
            if self.returning_home:
                self.get_logger().info('✅ Successfully arrived at Home Base!')
                self.save_map_and_exit()
            else:
                self.get_logger().info('Target reached successfully!')
        elif status == 6: # CANCELED (by our timeout handler)
            pass 
        else: # ABORTED or FAILED (e.g., dynamic obstacles)
            self.get_logger().warn('Navigation failed (Unexpected obstacle). Adding to Blacklist.')
            if self.returning_home:
                self.save_map_and_exit()
            else:
                self.blacklist_zones.append(self.current_target)

    # ==========================================
    # SHUTDOWN AND MAP SAVING
    # ==========================================
    def save_map_and_exit(self):
        self.get_logger().info("💾 SAVING MAP...")
        os.system("ros2 run nav2_map_server map_saver_cli -f ~/turtlebot_ws/explored_map")
        self.get_logger().info("✅ Map saved as 'explored_map' in ~/turtlebot_ws/")
        self.get_logger().info("Shutting down node.")
        raise SystemExit

# ==========================================
# MAIN ENTRY POINT
# ==========================================
def main(args=None):
    rclpy.init(args=args)             
    node = UltimateExplorer()         
    try:
        rclpy.spin(node)              
    except SystemExit:
        pass                          # Graceful exit without error tracebacks
    node.destroy_node()               
    rclpy.shutdown()                  

if __name__ == '__main__':
    main()
