#!/usr/bin/env python3
"""
Thruster Visualization for Hydrus Submarine

This script subscribes to thruster topics and visualizes their current values
in a terminal-based ASCII diagram.

Color coding:
- Regular thrusters: cyan
- Depth thrusters: green
- Torpedo thrusters: yellow
"""

import os
import time
import threading
import rospy
from std_msgs.msg import Int16
from termcolor import colored

class ThrusterVisualizer:
    """Visualizes thruster values in an ASCII diagram"""
    
    def __init__(self):
        rospy.init_node('thruster_visualizer', anonymous=True)
        
        # Store thruster values (PWM values: 1000-2000)
        self.thrusters = {i: 1500 for i in range(1, 9)}
        
        # Define thruster types for color coding
        self.regular_thrusters = [1, 4, 5, 8]  # Regular thrusters (cyan)
        self.depth_thrusters = [3, 6]          # Depth thrusters (green)
        self.torpedo_thrusters = [2, 7]        # Torpedo thrusters (yellow)
        
        # Set up subscribers for individual thrusters
        for i in range(1, 9):
            rospy.Subscriber(f"/hydrus/thrusters/{i}", Int16, 
                             lambda msg, idx=i: self.thruster_callback(idx, msg))
        
        # Subscribe to depth and torpedo topics to show combined values
        rospy.Subscriber("/hydrus/depth", Int16, self.depth_callback)
        rospy.Subscriber("/hydrus/torpedo", Int16, self.torpedo_callback)
        
        # Set up display parameters
        self.update_interval = 0.1  # seconds
        self.running = True
        self.depth_value = 1500
        self.torpedo_value = 1500
        
        # Start display thread
        self.display_thread = threading.Thread(target=self._display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()
        
    def thruster_callback(self, thruster_id, msg):
        """Handle thruster value updates"""
        self.thrusters[thruster_id] = msg.data
    
    def depth_callback(self, msg):
        """Handle depth motor value updates"""
        self.depth_value = msg.data
    
    def torpedo_callback(self, msg):
        """Handle torpedo value updates"""
        self.torpedo_value = msg.data
    
    def _get_thruster_str(self, thruster_id):
        """Format a thruster value with appropriate color and direction indicator"""
        value = self.thrusters[thruster_id]
        
        # Determine direction indicator
        direction = "○"  # Neutral
        if value > 1510:
            direction = "►"  # Forward
        elif value < 1490:
            direction = "◄"  # Reverse
            
        # Format string with PWM value
        value_str = f"{thruster_id}{direction} [{value}]"
        
        # Apply appropriate color based on thruster type
        if thruster_id in self.regular_thrusters:
            return colored(value_str, "cyan")
        elif thruster_id in self.depth_thrusters:
            return colored(value_str, "green")
        else:  # torpedo thrusters
            return colored(value_str, "yellow")
    
    def _display_loop(self):
        """Continuously update the ASCII diagram with current values"""
        while self.running and not rospy.is_shutdown():
            self._display_thrusters()
            time.sleep(self.update_interval)
    
    def _display_thrusters(self):
        """Display the ASCII diagram with current thruster values"""
        # Clear the terminal
        os.system('clear')
        
        # Print header
        print(colored("=== HYDRUS SUBMARINE THRUSTER VISUALIZATION ===", "white", attrs=["bold"]))
        print(colored("Regular thrusters: ", "cyan") + 
              colored("Depth thrusters: ", "green") + 
              colored("Torpedo thrusters: ", "yellow"))
        print(colored(f"Depth command: {self.depth_value}", "green") + 
              colored(f" | Torpedo command: {self.torpedo_value}", "yellow"))
        print("")
        
        # Generate ASCII diagram with thruster values
        diagram = [
            f"{self._get_thruster_str(1)}           {self._get_thruster_str(5)}",
            "     \\          /",
            "      |________|        ",
            f"{self._get_thruster_str(2)}--|        |--{self._get_thruster_str(6)}",
            "      |        |",
            "      |        |",
            f"{self._get_thruster_str(3)}--|________|--{self._get_thruster_str(7)}", 
            "      |        |",
            f"{self._get_thruster_str(4)}/          \\{self._get_thruster_str(8)}"
        ]
        
        # Print diagram
        for line in diagram:
            print(line)
        
        print("\nPress Ctrl+C to exit")
    
    def shutdown(self):
        """Clean shutdown procedure"""
        self.running = False
        time.sleep(0.2)  # Give display thread time to finish


def main():
    try:
        visualizer = ThrusterVisualizer()
        
        # Keep the node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nShutting down thruster visualizer...")
    finally:
        if 'visualizer' in locals():
            visualizer.shutdown()


if __name__ == "__main__":
    main()