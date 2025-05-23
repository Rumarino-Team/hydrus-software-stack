#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32, Int16MultiArray
from autonomy.msg import NavigateToWaypointFeedback
import time
import colorama
from colorama import Fore, Back, Style

class ControllerMonitor:
    """
    Monitor the submarine controller's state, including active movement phase
    and remaining distance to complete each phase.
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('controller_monitor', anonymous=True)
        colorama.init()
        
        # State variables
        self.submarine_pose = None
        self.target_point = None
        self.moving_state = [False, False, False]  # [depth, rotation, linear]
        self.last_state_change = time.time()
        self.delta = 0.05  # Default threshold (can be updated from controller)
        self.distance_to_target = float('inf')
        self.depth_distance = float('inf')
        self.rotation_distance = float('inf')
        self.linear_distance = float('inf')
        self.state_names = ["DEPTH", "ROTATION", "LINEAR"]
        self.active_state_index = 0
        
        # Subscribers
        rospy.Subscriber('/zed2i/zed_node/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/controller/target_point', Point, self.target_callback)
        rospy.Subscriber('/controller/moving_state', Int16MultiArray, self.state_callback)
        rospy.Subscriber('/controller/delta', Float32, self.delta_callback)
        
        # Set up display refresh rate
        self.rate = rospy.Rate(5)  # 5 Hz refresh rate
        
        # If the controller isn't publishing these topics yet, create them
        self.check_and_create_publishers()

    def check_and_create_publishers(self):
        """Create publishers for controller state info if they don't exist"""
        # This is a temporary solution - ideally the controller would publish this information
        self.target_pub = rospy.Publisher('/controller/target_point', Point, queue_size=10)
        self.state_pub = rospy.Publisher('/controller/moving_state', Int16MultiArray, queue_size=10)
        self.delta_pub = rospy.Publisher('/controller/delta', Float32, queue_size=10)

    def pose_callback(self, msg):
        """Store the current submarine pose"""
        self.submarine_pose = msg
        self.update_distances()

    def target_callback(self, msg):
        """Store the current target point"""
        self.target_point = msg
        self.update_distances()

    def state_callback(self, msg):
        """Update the movement state array"""
        self.moving_state = [bool(val) for val in msg.data]
        self.active_state_index = self.moving_state.index(True) if True in self.moving_state else -1
        self.last_state_change = time.time()

    def delta_callback(self, msg):
        """Update the delta threshold value"""
        self.delta = msg.data
        
    def calculate_yaw_to_target(self, current_position, target_position):
        """Calculate yaw angle to target position"""
        dx = target_position.x - current_position.x
        dy = target_position.y - current_position.y
        return math.atan2(dy, dx)

    def calculate_current_yaw(self, orientation):
        """Calculate current yaw from quaternion"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def update_distances(self):
        """Calculate distances for each movement phase"""
        if self.submarine_pose is None or self.target_point is None:
            return
            
        pos = self.submarine_pose.pose.position
        
        # Calculate overall distance to target
        self.distance_to_target = math.sqrt(
            (pos.x - self.target_point.x) ** 2 +
            (pos.y - self.target_point.y) ** 2 +
            (pos.z - self.target_point.z) ** 2
        )
        
        # Calculate depth distance
        self.depth_distance = abs(pos.z - self.target_point.z)
        
        # Calculate rotation distance (angle difference)
        target_yaw = self.calculate_yaw_to_target(pos, self.target_point)
        current_yaw = self.calculate_current_yaw(self.submarine_pose.pose.orientation)
        angle_diff = self.normalize_angle(target_yaw - current_yaw)
        self.rotation_distance = abs(angle_diff)
        
        # Calculate linear distance (in x-y plane)
        self.linear_distance = math.sqrt(
            (pos.x - self.target_point.x) ** 2 +
            (pos.y - self.target_point.y) ** 2
        )

    def display_monitor(self):
        """Display the controller monitoring information"""
        # Clear terminal
        print("\033c", end="")
        
        # Display header
        print(f"{Fore.CYAN}{'=' * 60}")
        print(f"{Fore.CYAN}{'SUBMARINE CONTROLLER MONITOR':^60}")
        print(f"{Fore.CYAN}{'=' * 60}{Style.RESET_ALL}")
        
        # Display current state
        state_str = f"UNKNOWN"
        if self.active_state_index >= 0 and self.active_state_index < len(self.state_names):
            state_str = self.state_names[self.active_state_index]
        
        print(f"{Fore.YELLOW}ACTIVE MOVEMENT STATE:{Style.RESET_ALL} {Fore.GREEN}{state_str}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}STATE DURATION:{Style.RESET_ALL} {time.time() - self.last_state_change:.1f} seconds")
        
        # Display DELTA threshold
        print(f"{Fore.YELLOW}DELTA THRESHOLD:{Style.RESET_ALL} {self.delta:.4f}")
        
        # Display current position
        if self.submarine_pose:
            pos = self.submarine_pose.pose.position
            print(f"\n{Fore.YELLOW}SUBMARINE POSITION:{Style.RESET_ALL}")
            print(f"  X: {pos.x:.3f}  Y: {pos.y:.3f}  Z: {pos.z:.3f}")
            
            # Get yaw in degrees
            yaw_deg = math.degrees(self.calculate_current_yaw(self.submarine_pose.pose.orientation))
            print(f"  YAW: {yaw_deg:.1f}°")
            
        # Display target point
        if self.target_point:
            print(f"\n{Fore.YELLOW}TARGET POSITION:{Style.RESET_ALL}")
            print(f"  X: {self.target_point.x:.3f}  Y: {self.target_point.y:.3f}  Z: {self.target_point.z:.3f}")
        
        # Display distances
        print(f"\n{Fore.YELLOW}MOVEMENT PHASE STATUS:{Style.RESET_ALL}")
        
        # Depth phase
        depth_status = "COMPLETE" if self.depth_distance <= self.delta else "ACTIVE" if self.moving_state[0] else "PENDING"
        depth_color = Fore.GREEN if depth_status == "COMPLETE" else Fore.CYAN if depth_status == "ACTIVE" else Fore.WHITE
        print(f"  DEPTH PHASE:    {depth_color}{depth_status}{Style.RESET_ALL}")
        print(f"    Distance: {self.depth_distance:.4f} m")
        print(f"    Remaining: {max(0, self.depth_distance - self.delta):.4f} m")
        
        # Rotation phase
        rotation_status = "COMPLETE" if self.rotation_distance <= self.delta else "ACTIVE" if self.moving_state[1] else "PENDING"
        rotation_color = Fore.GREEN if rotation_status == "COMPLETE" else Fore.CYAN if rotation_status == "ACTIVE" else Fore.WHITE
        print(f"  ROTATION PHASE: {rotation_color}{rotation_status}{Style.RESET_ALL}")
        print(f"    Angle diff: {math.degrees(self.rotation_distance):.1f}°")
        print(f"    Remaining: {max(0, math.degrees(self.rotation_distance - self.delta)):.1f}°")
        
        # Linear phase
        linear_status = "COMPLETE" if self.linear_distance <= self.delta else "ACTIVE" if self.moving_state[2] else "PENDING"
        linear_color = Fore.GREEN if linear_status == "COMPLETE" else Fore.CYAN if linear_status == "ACTIVE" else Fore.WHITE
        print(f"  LINEAR PHASE:   {linear_color}{linear_status}{Style.RESET_ALL}")
        print(f"    Distance: {self.linear_distance:.4f} m")
        print(f"    Remaining: {max(0, self.linear_distance - self.delta):.4f} m")
        
        # Overall distance
        print(f"\n{Fore.YELLOW}OVERALL STATUS:{Style.RESET_ALL}")
        overall_status = "COMPLETE" if self.distance_to_target <= self.delta else "IN PROGRESS"
        overall_color = Fore.GREEN if overall_status == "COMPLETE" else Fore.CYAN
        print(f"  STATUS: {overall_color}{overall_status}{Style.RESET_ALL}")
        print(f"  Total Distance: {self.distance_to_target:.4f} m")
        print(f"  Completion: {min(100, max(0, (1 - (self.distance_to_target / (self.depth_distance + self.linear_distance + 0.001))) * 100)):.1f}%")

        # Help information at bottom
        print(f"\n{Fore.YELLOW}NOTE:{Style.RESET_ALL} If controller never stops updating, try increasing DELTA in controllers.py")
        print(f"{Fore.YELLOW}DELTA location:{Style.RESET_ALL} controllers.py -> Constants class -> DELTA attribute (currently {self.delta})")

    def run(self):
        """Main run loop"""
        print("Starting Controller Monitor...")
        print("Waiting for controller data...")
        
        while not rospy.is_shutdown():
            self.display_monitor()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        monitor = ControllerMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass