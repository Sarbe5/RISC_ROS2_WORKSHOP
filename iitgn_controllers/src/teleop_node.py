#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import sys
import termios
import tty
import select
import threading
import time
from builtin_interfaces.msg import Time as TimeMsg

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_position_command', 10)
        self.config_pub = self.create_publisher(String, '/joint_position_command_config', 10)
        
        # Parameters
        self.declare_parameter('joint_step', 0.01)  # radians per keypress
        self.declare_parameter('num_joints', 7)
        self.declare_parameter('direct_tracking', True)
        
        self.joint_step = self.get_parameter('joint_step').value
        self.num_joints = self.get_parameter('num_joints').value
        self.direct_tracking = self.get_parameter('direct_tracking').value
        
        # Initialize joint positions to Franka FR3 ready position (safe configuration)
        self.franka_ready_position = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Ready pose
        self.franka_home_position = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]   # Home pose
        
        # Additional Franka poses
        self.franka_poses = {
            'ready': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            'candle': [0.0, -0.785, 0.0, -2.356, 0.0, 3.927, 0.785],  # Upright position
            'transport': [0.0, -1.143, 0.0, -2.618, 0.0, 1.571, 0.785],  # Safe transport
            'inspect': [0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 0.785],  # Table inspection
        }
        
        self.current_joint_positions = self.franka_ready_position.copy()
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        self.running = True
        
        # Start keyboard thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        
        self.print_instructions()
        self.get_logger().info('Initialized with Franka FR3 ready position')
        self.print_current_joints()
        
    def __del__(self):
        self.restore_terminal()
        
    def restore_terminal(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        except:
            pass
    
    def get_key(self):
        """Get a single keypress without blocking"""
        tty.setraw(sys.stdin.fileno())
        try:
            if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                key = sys.stdin.read(1)
                return key
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def print_instructions(self):
        print("\n" + "="*50)
        print("    FRANKA ROBOT KEYBOARD TELEOP")
        print("="*50)
        print("Joint Control (numbers increase, shift+numbers decrease):")
        print("  1/! : Joint 1    2/@ : Joint 2    3/# : Joint 3")
        print("  4/$ : Joint 4    5/% : Joint 5    6/^ : Joint 6    7/& : Joint 7")
        print("\nAlternative controls:")
        print("  q/a : Joint 1    w/s : Joint 2    e/d : Joint 3")
        print("  r/f : Joint 4    t/g : Joint 5    y/h : Joint 6    u/j : Joint 7")
        print("\nCommands:")
        print("  SPACE : Franka home position    x : Stop/Emergency stop")
        print("  z : Toggle direct tracking    0 : Reset to ready position")
        print("  +/= : Increase step    -/_ : Decrease step")
        print("  i : Show current joint values    c : Show predefined poses")
        print("  ESC : Quit")
        print(f"\nStep size: {self.joint_step:.4f} rad")
        print(f"Direct tracking: {'ON' if self.direct_tracking else 'OFF'}")
        print("="*50 + "\n")
        print("Ready for input...")
    
    def keyboard_loop(self):
        while self.running:
            key = self.get_key()
            if key:
                self.process_key(key)
            time.sleep(0.01)  # Small delay to prevent high CPU usage
    
    def process_key(self, key):
        send_joint_command = False
        send_config_command = False
        config_msg = ""
        
        # Joint control with numbers (increase) and shift+numbers (decrease)
        joint_controls = {
            # Increase joints
            '1': (0, self.joint_step),
            '2': (1, self.joint_step),
            '3': (2, self.joint_step),
            '4': (3, self.joint_step),
            '5': (4, self.joint_step),
            '6': (5, self.joint_step),
            '7': (6, self.joint_step),
            
            # Decrease joints
            '!': (0, -self.joint_step),
            '@': (1, -self.joint_step),
            '#': (2, -self.joint_step),
            '$': (3, -self.joint_step),
            '%': (4, -self.joint_step),
            '^': (5, -self.joint_step),
            '&': (6, -self.joint_step),
            
            # Alternative QWERTY controls (easier to use)
            'q': (0, self.joint_step),
            'a': (0, -self.joint_step),
            'w': (1, self.joint_step),
            's': (1, -self.joint_step),
            'e': (2, self.joint_step),
            'd': (2, -self.joint_step),
            'r': (3, self.joint_step),
            'f': (3, -self.joint_step),
            't': (4, self.joint_step),
            'g': (4, -self.joint_step),
            'y': (5, self.joint_step),
            'h': (5, -self.joint_step),
            'u': (6, self.joint_step),
            'j': (6, -self.joint_step),
        }
        
        if key in joint_controls:
            joint_idx, delta = joint_controls[key]
            self.current_joint_positions[joint_idx] += delta
            send_joint_command = True
            self.get_logger().info(f'Joint {joint_idx + 1}: {self.current_joint_positions[joint_idx]:.3f} rad')
            
        elif key == ' ':  # Space for home
            self.current_joint_positions = self.franka_home_position.copy()
            send_joint_command = True
            config_msg = "home"
            send_config_command = True
            self.get_logger().info('Moving to Franka home position...')
            
        elif key in ['x', 'X']:  # Stop
            config_msg = "stop"
            send_config_command = True
            self.get_logger().info('Emergency stop!')
            
        elif key in ['z', 'Z']:  # Toggle direct tracking
            self.direct_tracking = not self.direct_tracking
            config_msg = "direct_tracking_on" if self.direct_tracking else "direct_tracking_off"
            send_config_command = True
            self.get_logger().info(f'Direct tracking: {"ON" if self.direct_tracking else "OFF"}')
            
        elif key == '0':  # Reset to ready position (not zero!)
            self.current_joint_positions = self.franka_ready_position.copy()
            send_joint_command = True
            self.get_logger().info('Reset to Franka ready position')
            
        elif key in ['+', '=']:  # Increase step size
            self.joint_step = min(self.joint_step * 1.5, 0.5)
            self.get_logger().info(f'Step size: {self.joint_step:.4f} rad')
            
        elif key in ['-', '_']:  # Decrease step size
            self.joint_step = max(self.joint_step / 1.5, 0.001)
            self.get_logger().info(f'Step size: {self.joint_step:.4f} rad')
            
        elif key == '\x1b':  # ESC key
            self.get_logger().info('Shutting down...')
            self.running = False
            rclpy.shutdown()
            return
            
        elif key in ['?', 'H'] and not send_joint_command:  # Help
            self.print_instructions()
            
        elif key in ['i', 'I']:  # Show current joint values
            self.print_current_joints()
            
        elif key in ['c', 'C']:  # Show predefined poses
            self.print_predefined_poses()
            
        elif key.isdigit() and int(key) >= 1 and int(key) <= 4 and ord(key) > ord('7'):  # Poses 8,9 mapped to poses
            # This is for future pose selection, but keeping simple for now
            pass
        
        # Send joint command
        if send_joint_command:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = self.current_joint_positions.copy()
            
            # Set direct tracking mode if enabled
            if self.direct_tracking:
                msg.name = ["direct_track"]
            
            self.joint_cmd_pub.publish(msg)
        
        # Send config command
        if send_config_command:
            msg = String()
            msg.data = config_msg
            self.config_pub.publish(msg)
    
    def print_current_joints(self):
        """Display current joint positions"""
        print("\n" + "="*30)
        print("   CURRENT JOINT POSITIONS")
        print("="*30)
        joint_names = ['J1 (Base)', 'J2 (Shoulder)', 'J3 (Arm)', 'J4 (Elbow)', 
                      'J5 (Forearm)', 'J6 (Wrist)', 'J7 (Flange)']
        for i, (name, pos) in enumerate(zip(joint_names, self.current_joint_positions)):
            print(f"{name:15}: {pos:8.3f} rad ({pos*180/3.14159:7.1f}°)")
        print("="*30 + "\n")
    
    def print_predefined_poses(self):
        """Display available predefined poses"""
        print("\n" + "="*40)
        print("      FRANKA FR3 PREDEFINED POSES")
        print("="*40)
        for name, joints in self.franka_poses.items():
            print(f"{name.capitalize():12}: {joints}")
        print("="*40 + "\n")
        print("Note: Currently only 'ready' and 'home' are accessible via keys")
        print("Use SPACE for home, '0' for ready position\n")


class SafeTeleopNode(KeyboardTeleopNode):
    """Enhanced version with additional safety features"""
    
    def __init__(self):
        super().__init__()
        
        # Franka FR3 joint limits (in radians)
        self.declare_parameter('max_joint_limits', [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.declare_parameter('min_joint_limits', [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        
        self.max_limits = self.get_parameter('max_joint_limits').value
        self.min_limits = self.get_parameter('min_joint_limits').value
        
        # Safety timer - stop if no input for too long
        self.last_command_time = time.time()
        self.safety_timeout = 30.0  # seconds
        
        # Create timer for safety monitoring
        self.safety_timer = self.create_timer(1.0, self.safety_check)
    
    def process_key(self, key):
        self.last_command_time = time.time()
        
        # Store original positions for validation
        original_positions = self.current_joint_positions.copy()
        
        # Process the key normally
        super().process_key(key)
        
        # Check joint limits after processing
        if self.validate_joint_limits():
            # Limits OK, proceed normally
            pass
        else:
            # Revert to original positions
            self.current_joint_positions = original_positions
            self.get_logger().warn('Command rejected: would exceed joint limits')
    
    def validate_joint_limits(self):
        """Check if current joint positions are within limits"""
        for i, pos in enumerate(self.current_joint_positions):
            if pos < self.min_limits[i] or pos > self.max_limits[i]:
                return False
        return True
    
    def safety_check(self):
        """Monitor for safety timeout"""
        if time.time() - self.last_command_time > self.safety_timeout:
            self.get_logger().warn('Safety timeout - no commands received recently')
            # Could send stop command here if desired
            # config_msg = String()
            # config_msg.data = "stop"
            # self.config_pub.publish(config_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Choose between basic and safe version
        # node = KeyboardTeleopNode()
        node = SafeTeleopNode()
        
        print("Keyboard teleop node started. Press '?' for help.")
        
        # Spin in the main thread
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        try:
            node.restore_terminal()
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()