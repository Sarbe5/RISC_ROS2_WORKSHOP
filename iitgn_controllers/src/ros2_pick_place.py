#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class FrankaPickPlace(Node):
    def __init__(self):
        super().__init__('franka_pick_place')
        
        # Create callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Publisher for joint positions
        self.joint_pub = self.create_publisher(
            JointState, 
            '/joint_position_command', 
            10,
            callback_group=self.callback_group
        )
        
        # Publisher for gripper control using effort controller
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_effort_controller/commands',
            10,
            callback_group=self.callback_group
        )
        
        # Subscriber for joint states to monitor position
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Current joint positions
        self.current_q = None
        self.target_q = None
        self.position_tolerance = 0.02  # 2cm tolerance
        
        self.get_logger().info('Gripper effort controller publisher created!')
        self.get_logger().info('Joint state subscriber created!')
        
        # Define robot positions (joint angles in radians) - optimized for better pick/place
        self.positions = {
            'home': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            
            # Pick sequence positions
            'pick_ready': [0.2, -0.3, -0.1, -2.0, 0.1, 1.7, 0.785],      # Ready position near pick area
            'pick_approach': [0.2, 0.1, -0.1, -1.5, 0.1, 1.6, 0.785],    # Above object (higher)
            'pick_position': [0.2, 0.3, -0.1, -1.2, 0.1, 1.5, 0.785],    # At object level (lower)
            
            # Place sequence positions  
            'place_ready': [1.0, -0.3, 0.3, -2.0, -0.1, 1.7, 0.785],     # Ready position near place area
            'place_approach': [1.0, 0.1, 0.3, -1.5, -0.1, 1.6, 0.785],   # Above drop location
            'place_position': [1.0, 0.3, 0.3, -1.2, -0.1, 1.5, 0.785]    # At drop level
        }
        
        # Gripper effort values
        self.gripper_open_effort = 5.0      # Positive value to open
        self.gripper_close_effort = -5.0    # Negative value to close
        
        # Movement timing
        self.movement_time = 5.0
        self.gripper_time = 1.0
        self.max_wait_time = 10.0  # Maximum time to wait for position
        
    def joint_state_callback(self, msg):
        """Callback to track current joint positions"""
        if len(msg.position) >= 7:
            self.current_q = list(msg.position[:7])
            
    def wait_for_position_reached(self, target_position_name, timeout=None):
        """Wait until robot reaches the target position"""
        if self.current_q is None:
            self.get_logger().warn('No joint states received yet, using time-based wait')
            self.wait_for_movement()
            return True
            
        target_q = self.positions[target_position_name]
        timeout = timeout or self.max_wait_time
        start_time = time.time()
        
        self.get_logger().info(f'Waiting for position: {target_position_name}')
        
        while time.time() - start_time < timeout:
            if self.current_q is not None:
                # Calculate position error
                error = sum(abs(current - target) for current, target in zip(self.current_q, target_q))
                
                if error < self.position_tolerance:
                    self.get_logger().info(f'Position reached! Error: {error:.4f}')
                    return True
                    
            time.sleep(0.1)  # Check every 100ms
            
        self.get_logger().warn(f'Position timeout after {timeout}s, continuing anyway')
        return False
        
    def publish_joint_position(self, position_name, wait_for_completion=True):
        """Publish joint position command and optionally wait for completion"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
                   'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
        msg.position = self.positions[position_name]
        
        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published position: {position_name}')
        
        if wait_for_completion:
            return self.wait_for_position_reached(position_name)
        
    def control_gripper(self, effort_value):
        """Control gripper using effort controller topic"""
        msg = Float64MultiArray()
        msg.data = [effort_value]
        
        self.gripper_pub.publish(msg)
        
        if effort_value > 0:
            self.get_logger().info(f'Opening gripper with effort: {effort_value}')
        else:
            self.get_logger().info(f'Closing gripper with effort: {effort_value}')
        
        # Wait for gripper action to complete
        time.sleep(self.gripper_time)
        
    def wait_for_movement(self, duration=None):
        """Wait for robot movement to complete"""
        wait_time = duration or self.movement_time
        self.get_logger().info(f'Waiting {wait_time} seconds for movement...')
        time.sleep(wait_time)
        
    def execute_pick_and_place(self):
        """Execute the complete pick and place sequence"""
        self.get_logger().info('Starting pick and place sequence...')
        
        try:
            # Step 1: Go to home position
            self.get_logger().info('Step 1: Moving to home position')
            self.publish_joint_position('home')
            
            # Step 2: Open gripper at start
            self.get_logger().info('Step 2: Opening gripper')
            self.control_gripper(self.gripper_open_effort)
            
            # Step 3: Move to pick ready position
            self.get_logger().info('Step 3: Moving to pick ready position')
            self.publish_joint_position('pick_ready')
            
            # Step 4: Move to pick approach (above object)
            self.get_logger().info('Step 4: Moving to pick approach position')
            self.publish_joint_position('pick_approach')
            
            # Step 5: Move down to pick position
            self.get_logger().info('Step 5: Moving down to pick position')
            self.publish_joint_position('pick_position')
            
            # Step 6: Close gripper to grasp object
            self.get_logger().info('Step 6: Grasping object')
            self.control_gripper(self.gripper_close_effort)
            
            # Step 7: Move back up to approach position
            self.get_logger().info('Step 7: Lifting object')
            self.publish_joint_position('pick_approach')
            
            # Step 8: Move to pick ready position (intermediate)
            self.get_logger().info('Step 8: Moving through pick ready position')
            self.publish_joint_position('pick_ready')
            
            # Step 9: Move to place ready position
            self.get_logger().info('Step 9: Moving to place ready position')
            self.publish_joint_position('place_ready')
            
            # Step 10: Move to place approach position
            self.get_logger().info('Step 10: Moving to place approach position')
            self.publish_joint_position('place_approach')
            
            # Step 11: Move down to place position
            self.get_logger().info('Step 11: Moving down to place position')
            self.publish_joint_position('place_position')
            
            # Step 12: Open gripper to drop object
            self.get_logger().info('Step 12: Dropping object')
            self.control_gripper(self.gripper_open_effort)
            
            # Step 13: Move back up to approach
            self.get_logger().info('Step 13: Moving up from drop position')
            self.publish_joint_position('place_approach')
            
            # Step 14: Move to place ready position
            self.get_logger().info('Step 14: Moving to place ready position')
            self.publish_joint_position('place_ready')
            
            # Step 15: Return to home
            self.get_logger().info('Step 15: Returning to home position')
            self.publish_joint_position('home')
            
            self.get_logger().info('Pick and place sequence completed successfully!')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Pick and place failed: {str(e)}')
            # Emergency stop - open gripper
            self.get_logger().info('Emergency: Opening gripper')
            self.control_gripper(self.gripper_open_effort)
            return False

def main(args=None):
    rclpy.init(args=args)
    
    node = FrankaPickPlace()
    
    # Use MultiThreadedExecutor for concurrent operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Start the pick and place sequence
        node.get_logger().info('Franka Pick and Place Node started')
        
        # Wait a moment for everything to initialize
        time.sleep(2.0)
        
        # Execute pick and place
        success = node.execute_pick_and_place()
        
        if success:
            node.get_logger().info('Mission accomplished!')
        else:
            node.get_logger().error('Mission failed!')
            
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
        # Emergency stop - open gripper
        node.control_gripper(node.gripper_open_effort)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()