#!/usr/bin/env python3
# filepath: /home/cesar/Projects/hydrus-software-stack/autonomy/src/thruster_mapper.py

import rospy
import time
from std_msgs.msg import Int8
from termcolor import colored

class ThrusterMapper:
    """
    Utility to map thruster IDs to physical locations on the submarine.
    Uses the ASCII diagram for reference:
      
     1 *            * 5
        \          /
         |________|        
     2*--|        |--* 6
         |        |
         |        |
     3*--|________|--* 7 
         |        |
     4 */          \* 8
    """

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('thruster_mapper')

        # Create publishers for each thruster
        self.thruster_pubs = {}
        for i in range(1, 9):
            topic = f'/thrusters/{i}'
            self.thruster_pubs[i] = rospy.Publisher(topic, Int8, queue_size=10)

        # Wait for publishers to connect
        rospy.loginfo("Waiting for thruster publishers to connect...")
        time.sleep(2)
        rospy.loginfo("Ready to test thrusters!")

    def test_thruster(self, thruster_id, value=2):
        """Test a single thruster with a specific value."""
        msg = Int8()
        msg.data = value  # Use a moderate forward value
        
        # Print thruster being activated
        print(colored(f"\nActivating thruster {thruster_id} with value {value}", "green"))
        print(self.get_ascii_diagram(thruster_id))
        
        # Send command to thruster
        self.thruster_pubs[thruster_id].publish(msg)
        time.sleep(2)  # Keep thruster running for 2 seconds
        
        # Stop the thruster
        msg.data = 0
        self.thruster_pubs[thruster_id].publish(msg)
        
        # Ask user for confirmation
        result = input(colored("\nDid you observe which thruster was activated? (y/n): ", "yellow"))
        return result.lower() == 'y'

    def get_ascii_diagram(self, highlight_id=None):
        """Return ASCII diagram with highlighted thruster."""
        diagram = [
            "  1 *            * 5",
            "     \\          /",
            "      |________|        ",
            "  2*--|        |--* 6",
            "      |        |",
            "      |        |",
            "  3*--|________|--* 7 ",
            "      |        |",
            "  4 */          \\* 8"
        ]
        
        if highlight_id is not None:
            line_idx = {1: 0, 2: 3, 3: 6, 4: 8, 5: 0, 6: 3, 7: 6, 8: 8}
            line = diagram[line_idx[highlight_id]]
            
            # Replace the thruster number with a highlighted version
            highlighted = colored(str(highlight_id), "red", attrs=["bold"])
            if highlight_id <= 4:  # Left side
                diagram[line_idx[highlight_id]] = line.replace(f"{highlight_id} ", f"{highlighted} ")
            else:  # Right side
                diagram[line_idx[highlight_id]] = line.replace(f"{highlight_id}", f"{highlighted}")
        
        return "\n".join(diagram)

    def run_mapping_procedure(self):
        """Run the mapping procedure for all thrusters."""
        print(colored("\n=== THRUSTER MAPPING PROCEDURE ===", "blue", attrs=["bold"]))
        print(colored("This utility will help you map thruster IDs to their physical locations.", "blue"))
        print(colored("Each thruster will be activated one by one.", "blue"))
        print(colored("Observe which one moves and confirm its position according to the diagram:", "blue"))
        print(self.get_ascii_diagram())
        
        mapping = {}
        
        for i in range(1, 9):
            print(colored(f"\n--- Testing Thruster {i} ---", "cyan", attrs=["bold"]))
            
            success = self.test_thruster(i)
            if not success:
                print(colored("Let's try that again...", "yellow"))
                success = self.test_thruster(i)
            
            if success:
                position = input(colored(f"Enter the position (1-8) of the thruster that was activated: ", "yellow"))
                try:
                    position = int(position)
                    if 1 <= position <= 8:
                        mapping[i] = position
                        print(colored(f"Mapped thruster {i} to position {position}", "green"))
                    else:
                        print(colored("Invalid position. Please enter a number between 1 and 8.", "red"))
                        i -= 1  # Retry this thruster
                except ValueError:
                    print(colored("Invalid input. Please enter a number.", "red"))
                    i -= 1  # Retry this thruster
        
        # Display final mapping
        print(colored("\n=== FINAL THRUSTER MAPPING ===", "blue", attrs=["bold"]))
        print("ROS Thruster ID -> Physical Position")
        for ros_id, position in mapping.items():
            print(f"Thruster {ros_id} -> Position {position}")
        
        print(colored("\nUse this mapping to update your thruster configuration.", "green"))

    def run_individual_test(self):
        """Test thrusters individually based on user input."""
        print(colored("\n=== INDIVIDUAL THRUSTER TEST ===", "blue", attrs=["bold"]))
        print("Enter thruster ID (1-8) to test, or 'q' to quit")
        
        while not rospy.is_shutdown():
            cmd = input(colored("\nThruster ID to test (1-8, or 'q' to quit): ", "yellow"))
            
            if cmd.lower() == 'q':
                break
            
            try:
                thruster_id = int(cmd)
                if 1 <= thruster_id <= 8:
                    value = input(colored(f"Power value (-4 to 4, 0 for neutral): ", "yellow"))
                    try:
                        value = int(value)
                        if -4 <= value <= 4:
                            self.test_thruster(thruster_id, value)
                        else:
                            print(colored("Invalid power value. Please enter a number between -4 and 4.", "red"))
                    except ValueError:
                        print(colored("Invalid input. Please enter a number.", "red"))
                else:
                    print(colored("Invalid thruster ID. Please enter a number between 1 and 8.", "red"))
            except ValueError:
                print(colored("Invalid input. Please enter a number.", "red"))

def main():
    try:
        mapper = ThrusterMapper()
        
        print(colored("Choose operation mode:", "cyan"))
        print("1. Full mapping procedure")
        print("2. Test individual thrusters")
        
        choice = input(colored("Enter your choice (1 or 2): ", "yellow"))
        
        if choice == '1':
            mapper.run_mapping_procedure()
        elif choice == '2':
            mapper.run_individual_test()
        else:
            print(colored("Invalid choice. Exiting.", "red"))
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()