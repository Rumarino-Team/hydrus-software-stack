#!/usr/bin/env python3
# filepath: /home/cesar/Projects/hydrus-software-stack/autonomy/scripts/controller/submarine_teleop.py

import rospy
import sys
import tty
import termios
import threading
import signal
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from termcolor import colored

class SubmarineTeleop:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('submarine_teleop', anonymous=True)
        
        # Create publishers for each thruster
        self.thruster_pubs = {}
        for i in range(1, 9):
            topic = f'/thrusters/{i}'
            self.thruster_pubs[i] = rospy.Publisher(topic, Float32, queue_size=10)

        # Command velocity publisher (for visualization/debugging)
        self.cmd_vel_pub = rospy.Publisher('/submarine/cmd_vel', TwistStamped, queue_size=10)
        
        # Initialize thruster values (1500 is neutral)
        self.thruster_values = [1500 for _ in range(8)]
        
        # Control parameters
        self.speed_increment = 25  # Increment for thruster adjustment
        self.max_speed_delta = 200  # Max deviation from neutral
        
        # Define thruster mappings (adjust as per your configuration)
        # As per ASCII diagram:
        #  1 *            * 5
        #     \          /
        #      |________|        
        #  2*--|        |--* 6
        #      |        |
        #      |        |
        #  3*--|________|--* 7 
        #      |        |
        #  4 */          \* 8
        
        # Zero-indexed for Python (thruster IDs minus 1)
        self.front_motors = [0, 4]      # Thrusters 1, 5
        self.back_motors = [3, 7]       # Thrusters 4, 8
        self.depth_motors = [1, 6]      # Thrusters 2, 7
        self.torpedo_motors = [2, 5]    # Thrusters 3, 6
        
        # Control state
        self.running = True
        self.key_thread = None

    def start(self):
        """Start the teleop node"""
        self.display_instructions()
        
        # Start the keyboard listener thread
        self.key_thread = threading.Thread(target=self.get_key_loop)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        # Start the publishing thread
        rate = rospy.Rate(10)  # 10 Hz
        try:
            while not rospy.is_shutdown() and self.running:
                self.publish_thruster_values()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.stop()
    
    def display_instructions(self):
        """Display control instructions"""
        print(colored("\n=== SUBMARINE KEYBOARD TELEOPERATION ===", "blue", attrs=["bold"]))
        print(colored("Control the submarine using the following keys:", "cyan"))
        print(colored("  W/S : Move forward/backward", "yellow"))
        print(colored("  A/D : Rotate left/right (yaw)", "yellow"))
        print(colored("  I/K : Move up/down", "yellow"))
        print(colored("  Q   : Quit", "yellow"))
        print(colored("  Space : Stop all movement (neutral)", "yellow"))
        print(colored("\nPress any key to begin...", "green"))

    def get_key(self):
        """Get a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def get_key_loop(self):
        """Continuously get keypresses and process them"""
        while self.running:
            key = self.get_key()
            if key:
                self.process_key(key)
    
    def process_key(self, key):
        """Process a keypress and update thruster values"""
        key = key.lower()
        
        # Quit on 'q'
        if key == 'q':
            self.running = False
            rospy.signal_shutdown("User requested exit")
            return
            
        # Stop all thrusters on spacebar
        elif key == ' ':
            print(colored("STOP - All thrusters set to neutral", "red"))
            self.reset_thrusters()
            
        # Forward/Backward control (W/S)
        elif key == 'w':
            self.move_forward()
            print(colored("Moving FORWARD", "green"))
        elif key == 's':
            self.move_backward()
            print(colored("Moving BACKWARD", "green"))
            
        # Rotation control (A/D)
        elif key == 'a':
            self.rotate_left()
            print(colored("Rotating LEFT", "green"))
        elif key == 'd':
            self.rotate_right()
            print(colored("Rotating RIGHT", "green"))
            
        # Depth control (I/K)
        elif key == 'i':
            self.move_up()
            print(colored("Moving UP", "green"))
        elif key == 'k':
            self.move_down()
            print(colored("Moving DOWN", "green"))
    
    def reset_thrusters(self):
        """Set all thrusters to neutral"""
        for i in range(8):
            self.thruster_values[i] = 1500
    
    def move_forward(self):
        """Command submarine to move forward"""
        for motor_id in self.front_motors + self.back_motors:
            self.thruster_values[motor_id] = 1500 + self.speed_increment
    
    def move_backward(self):
        """Command submarine to move backward"""
        for motor_id in self.front_motors + self.back_motors:
            self.thruster_values[motor_id] = 1500 - self.speed_increment
    
    def rotate_left(self):
        """Command submarine to rotate left (counterclockwise)"""
        for motor_id in self.front_motors:
            if motor_id % 2 == 0:  # Left side thrusters
                self.thruster_values[motor_id] = 1500 - self.speed_increment
            else:  # Right side thrusters
                self.thruster_values[motor_id] = 1500 + self.speed_increment
                
        for motor_id in self.back_motors:
            if motor_id % 2 == 0:  # Left side thrusters
                self.thruster_values[motor_id] = 1500 - self.speed_increment
            else:  # Right side thrusters
                self.thruster_values[motor_id] = 1500 + self.speed_increment
    
    def rotate_right(self):
        """Command submarine to rotate right (clockwise)"""
        for motor_id in self.front_motors:
            if motor_id % 2 == 0:  # Left side thrusters
                self.thruster_values[motor_id] = 1500 + self.speed_increment
            else:  # Right side thrusters
                self.thruster_values[motor_id] = 1500 - self.speed_increment
                
        for motor_id in self.back_motors:
            if motor_id % 2 == 0:  # Left side thrusters
                self.thruster_values[motor_id] = 1500 + self.speed_increment
            else:  # Right side thrusters
                self.thruster_values[motor_id] = 1500 - self.speed_increment
    
    def move_up(self):
        """Command submarine to move up"""
        for motor_id in self.depth_motors:
            self.thruster_values[motor_id] = 1500 + self.speed_increment
    
    def move_down(self):
        """Command submarine to move down"""
        for motor_id in self.depth_motors:
            self.thruster_values[motor_id] = 1500 - self.speed_increment
    
    def publish_thruster_values(self):
        """Publish current thruster values to ROS"""
        for i, value in enumerate(self.thruster_values):
            msg = Float32()
            msg.data = value
            self.thruster_pubs[i+1].publish(msg)
        
        # Also publish twist for visualization
        self.publish_cmd_vel()
    
    def publish_cmd_vel(self):
        """Publish TwistStamped message for visualization and logging"""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "base_link"
        
        # Simple approximation of movement based on thruster values
        # Forward/backward
        forward_thrust = sum([self.thruster_values[i] - 1500 for i in self.front_motors + self.back_motors]) / len(self.front_motors + self.back_motors)
        twist_msg.twist.linear.x = forward_thrust / self.max_speed_delta
        
        # Up/down
        depth_thrust = sum([self.thruster_values[i] - 1500 for i in self.depth_motors]) / len(self.depth_motors)
        twist_msg.twist.linear.z = depth_thrust / self.max_speed_delta
        
        # Yaw rotation
        left_thrust = sum([self.thruster_values[i] - 1500 for i in [self.front_motors[0], self.back_motors[0]]])
        right_thrust = sum([self.thruster_values[i] - 1500 for i in [self.front_motors[1], self.back_motors[1]]])
        yaw_thrust = (right_thrust - left_thrust) / 4.0
        twist_msg.twist.angular.z = yaw_thrust / self.max_speed_delta
        
        self.cmd_vel_pub.publish(twist_msg)
    
    def stop(self):
        """Stop the teleop node cleanly"""
        self.running = False
        self.reset_thrusters()
        self.publish_thruster_values()
        print(colored("\nExiting submarine teleoperation...", "blue"))


def signal_handler(_, __):
    """Handle system signals for clean shutdown"""
    print(colored("\nReceived shutdown signal!", "red"))
    rospy.signal_shutdown("Received shutdown signal!")
    sys.exit(0)


def main():
    # Set up signal handler for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and start the teleop node
    teleop = SubmarineTeleop()
    teleop.start()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        # Restore terminal settings in case of crash
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)