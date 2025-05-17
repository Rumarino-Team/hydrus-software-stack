#!/usr/bin/env python3
"""
Interactive Color Filter Controller for cv_publishers.py

This script provides an interactive interface to adjust color filter parameters
in real-time through the ROS service `/detector/set_color_filter`.
It also allows selecting different YOLO models from a Docker volume.
"""

import rospy
import sys
import json
import os
import curses
from termcolor import colored
from autonomy.srv import SetColorFilter, SetColorFilterRequest
from autonomy.srv import SetYoloModel, SetYoloModelRequest
import numpy as np

# YOLO model directory constant
YOLO_MODEL_DIR = "/yolo_models"

class ColorFilterController:
    def __init__(self):
        rospy.init_node('color_filter_controller', anonymous=True)
        
        # Default parameters
        self.tolerance = 0.4
        self.min_confidence = 0.3
        self.min_area = 0.2
        self.rgb_range = [255, 0, 0]  # Default: Red
        self.current_yolo_model = "yolov8n.pt"  # Default YOLO model
        
        # Color presets
        self.presets = {
            'red': {'tolerance': 0.4, 'min_confidence': 0.3, 'min_area': 0.2, 'rgb_range': [255, 0, 0]},
            'green': {'tolerance': 0.4, 'min_confidence': 0.3, 'min_area': 0.2, 'rgb_range': [0, 255, 0]},
            'blue': {'tolerance': 0.4, 'min_confidence': 0.3, 'min_area': 0.2, 'rgb_range': [0, 0, 255]},
            'yellow': {'tolerance': 0.4, 'min_confidence': 0.3, 'min_area': 0.2, 'rgb_range': [255, 255, 0]},
        }
        
        # Create presets directory if it doesn't exist
        presets_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'color_presets')
        if not os.path.exists(presets_dir):
            os.makedirs(presets_dir)
        self.presets_dir = presets_dir
        
        # Load saved presets
        self.load_saved_presets()
        
        print("Waiting for color filter service...")
        self.service_name = '/detector/set_color_filter'
        try:
            rospy.wait_for_service(self.service_name, timeout=10)
            self.set_color_filter = rospy.ServiceProxy(self.service_name, SetColorFilter)
            print("Color filter service connected!")
        except rospy.ROSException:
            print(f"Service {self.service_name} not available after waiting. Starting anyway.")
            self.set_color_filter = None

        print("Waiting for YOLO model service...")
        self.yolo_service_name = '/detector/set_yolo_model'
        try:
            rospy.wait_for_service(self.yolo_service_name, timeout=10)
            self.set_yolo_model = rospy.ServiceProxy(self.yolo_service_name, SetYoloModel)
            print("YOLO model service connected!")
        except rospy.ROSException:
            print(f"Service {self.yolo_service_name} not available after waiting. Starting anyway.")
            self.set_yolo_model = None
            
        # Get available YOLO models
        self.yolo_models = self.get_available_yolo_models()

    def get_available_yolo_models(self):
        """Get a list of available YOLO model files in the models directory"""
        models = []
        if os.path.exists(YOLO_MODEL_DIR):
            for filename in os.listdir(YOLO_MODEL_DIR):
                if filename.endswith('.pt'):
                    models.append(filename)
        else:
            print(f"Warning: YOLO model directory {YOLO_MODEL_DIR} not found.")
        
        print(f"Found {len(models)} YOLO models: {', '.join(models)}")
        return models

    def switch_yolo_model(self, model_name):
        """Switch to a different YOLO model"""
        if self.set_yolo_model is None:
            print("YOLO model service not available. Cannot switch models.")
            return False
            
        if model_name not in self.yolo_models:
            print(f"Model '{model_name}' not found! Available models: {', '.join(self.yolo_models)}")
            return False
            
        try:
            request = SetYoloModelRequest()
            request.model_name = model_name
            
            response = self.set_yolo_model(request)
            if response.success:
                self.current_yolo_model = model_name
                print(f"YOLO model switched to: {model_name}")
                print(f"Response: {response.message}")
                return True
            else:
                print(f"Failed to switch YOLO model: {response.message}")
                return False
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def load_saved_presets(self):
        """Load saved color presets from files"""
        presets_dir = self.presets_dir
        if os.path.exists(presets_dir):
            for filename in os.listdir(presets_dir):
                if filename.endswith('.json'):
                    preset_name = filename[:-5]  # Remove .json extension
                    try:
                        with open(os.path.join(presets_dir, filename), 'r') as f:
                            preset_data = json.load(f)
                            self.presets[preset_name] = preset_data
                    except Exception as e:
                        print(f"Error loading preset {preset_name}: {e}")

    def save_preset(self, name):
        """Save current parameters as a preset"""
        preset_data = {
            'tolerance': self.tolerance,
            'min_confidence': self.min_confidence,
            'min_area': self.min_area,
            'rgb_range': self.rgb_range
        }
        
        # Save to presets dictionary
        self.presets[name] = preset_data
        
        # Save to file
        try:
            with open(os.path.join(self.presets_dir, f"{name}.json"), 'w') as f:
                json.dump(preset_data, f, indent=4)
            print(f"Preset '{name}' saved successfully!")
        except Exception as e:
            print(f"Error saving preset: {e}")

    def load_preset(self, name):
        """Load parameters from a preset"""
        if name in self.presets:
            preset = self.presets[name]
            self.tolerance = preset['tolerance']
            self.min_confidence = preset['min_confidence']
            self.min_area = preset['min_area']
            self.rgb_range = preset['rgb_range']
            print(f"Loaded preset '{name}'")
            return True
        else:
            print(f"Preset '{name}' not found!")
            return False

    def update_filter(self):
        """Send the current parameters to the ROS service"""
        if self.set_color_filter is None:
            print("Service not available. Parameters not updated.")
            return False
        
        try:
            request = SetColorFilterRequest()
            request.tolerance = self.tolerance
            request.min_confidence = self.min_confidence
            request.min_area = self.min_area
            request.rgb_range = self.rgb_range
            
            response = self.set_color_filter(request)
            if response.success:
                print(f"Parameters updated: {response.message}")
                return True
            else:
                print(f"Failed to update parameters: {response.message}")
                return False
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def print_rgb_color(self):
        """Print the current RGB color with actual color visualization"""
        r, g, b = self.rgb_range
        # Use terminal colors to visualize
        print(f"RGB: [{colored(r, 'red')}, {colored(g, 'green')}, {colored(b, 'blue')}]")

    def print_status(self):
        """Print the current filter parameters"""
        print("\n--- Current Color Filter Parameters ---")
        print(f"Tolerance: {self.tolerance}")
        print(f"Min Confidence: {self.min_confidence}")
        print(f"Min Area: {self.min_area}")
        self.print_rgb_color()
        print("-------------------------------------\n")

    def print_help(self):
        """Print the help menu"""
        print("\n--- Color Filter Controller Help ---")
        print("Commands:")
        print("  help                   - Show this help menu")
        print("  status                 - Show current parameters")
        print("  set tolerance <value>  - Set tolerance (0.0-1.0)")
        print("  set confidence <value> - Set min confidence (0.0-1.0)")
        print("  set area <value>       - Set min area (0.0-1.0)")
        print("  set rgb <r> <g> <b>    - Set RGB color (0-255 for each)")
        print("  save <name>            - Save current settings as preset")
        print("  load <name>            - Load settings from preset")
        print("  list                   - List available presets")
        print("  update                 - Send current parameters to ROS")
        print("  yolo <model>           - Switch YOLO model")
        print("  exit/quit              - Exit the program")
        print("---------------------------------------\n")

    def run_interactive(self):
        """Run the controller in interactive mode"""
        self.print_help()
        
        while not rospy.is_shutdown():
            try:
                cmd = input("Filter Controller> ").strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == "exit" or cmd[0] == "quit":
                    break
                elif cmd[0] == "help":
                    self.print_help()
                elif cmd[0] == "status":
                    self.print_status()
                elif cmd[0] == "set" and len(cmd) >= 3:
                    if cmd[1] == "tolerance" and len(cmd) == 3:
                        try:
                            value = float(cmd[2])
                            if 0 <= value <= 1:
                                self.tolerance = value
                                print(f"Tolerance set to {value}")
                            else:
                                print("Tolerance must be between 0 and 1")
                        except ValueError:
                            print("Invalid value. Must be a number between 0 and 1")
                    elif cmd[1] == "confidence" and len(cmd) == 3:
                        try:
                            value = float(cmd[2])
                            if 0 <= value <= 1:
                                self.min_confidence = value
                                print(f"Min confidence set to {value}")
                            else:
                                print("Min confidence must be between 0 and 1")
                        except ValueError:
                            print("Invalid value. Must be a number between 0 and 1")
                    elif cmd[1] == "area" and len(cmd) == 3:
                        try:
                            value = float(cmd[2])
                            if 0 <= value <= 1:
                                self.min_area = value
                                print(f"Min area set to {value}")
                            else:
                                print("Min area must be between 0 and 1")
                        except ValueError:
                            print("Invalid value. Must be a number between 0 and 1")
                    elif cmd[1] == "rgb" and len(cmd) == 5:
                        try:
                            r = int(cmd[2])
                            g = int(cmd[3])
                            b = int(cmd[4])
                            if 0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255:
                                self.rgb_range = [r, g, b]
                                print(f"RGB set to [{r}, {g}, {b}]")
                                self.print_rgb_color()
                            else:
                                print("RGB values must be between 0 and 255")
                        except ValueError:
                            print("Invalid RGB values. Must be integers between 0 and 255")
                    else:
                        print("Invalid 'set' command. Type 'help' for usage.")
                elif cmd[0] == "save" and len(cmd) == 2:
                    self.save_preset(cmd[1])
                elif cmd[0] == "load" and len(cmd) == 2:
                    if self.load_preset(cmd[1]):
                        self.update_filter()
                elif cmd[0] == "list":
                    print("\nAvailable presets:")
                    for name in sorted(self.presets.keys()):
                        preset = self.presets[name]
                        r, g, b = preset['rgb_range']
                        print(f"  {name}: RGB[{r},{g},{b}], Tolerance={preset['tolerance']}")
                    print()
                elif cmd[0] == "update":
                    self.update_filter()
                elif cmd[0] == "yolo" and len(cmd) == 2:
                    self.switch_yolo_model(cmd[1])
                else:
                    print("Unknown command. Type 'help' for usage.")
            
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")

def run_curses_interface(stdscr, controller):
    """Run an advanced curses-based interface"""
    # Setup colors
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_BLUE, curses.COLOR_BLACK)
    
    # Hide cursor
    curses.curs_set(0)
    
    # Get screen dimensions
    height, width = stdscr.getmaxyx()
    
    # Create input window
    input_win = curses.newwin(3, width, height-3, 0)
    
    # Create parameter window
    param_win = curses.newwin(12, 40, 1, 1)
    
    # Create help window
    help_win = curses.newwin(12, 40, 1, 42)
    
    # Create status window
    status_win = curses.newwin(3, width, height-6, 0)
    
    # Create YOLO models window
    yolo_win = curses.newwin(8, width-2, 14, 1)
    
    # Main interface loop
    command = ""
    status_message = "Ready. Press ? for help."
    selected_param = 0  # 0:tolerance, 1:confidence, 2:area, 3:r, 4:g, 5:b
    param_names = ["Tolerance", "Min Confidence", "Min Area", "R", "G", "B"]
    
    # Refresh YOLO models
    controller.yolo_models = controller.get_available_yolo_models()
    
    while True:
        stdscr.clear()
        
        # Draw title
        stdscr.addstr(0, 0, "Color Filter Controller", curses.A_BOLD | curses.color_pair(2))
        
        # Update parameter window
        param_win.clear()
        param_win.box()
        param_win.addstr(0, 2, "Parameters", curses.A_BOLD)
        
        # Draw parameters with different colors for the selected one
        for i, name in enumerate(param_names):
            if i < 3:
                value = [controller.tolerance, controller.min_confidence, controller.min_area][i]
                if i == selected_param:
                    param_win.addstr(i+1, 2, f"> {name}: {value:.3f}", curses.color_pair(4) | curses.A_BOLD)
                else:
                    param_win.addstr(i+1, 2, f"  {name}: {value:.3f}")
            else:
                rgb_idx = i - 3
                value = controller.rgb_range[rgb_idx]
                rgb_color = [3, 2, 5][rgb_idx]  # Red, Green, Blue color pairs
                
                if i == selected_param:
                    param_win.addstr(i+1, 2, f"> {name}: {value}", curses.color_pair(4) | curses.A_BOLD)
                else:
                    param_win.addstr(i+1, 2, f"  {name}: {value}", curses.color_pair(rgb_color))
        
        # Current RGB color box
        r, g, b = controller.rgb_range
        color_text = f"Current Color: RGB({r},{g},{b})"
        param_win.addstr(8, 2, color_text)
        
        # Current YOLO model
        param_win.addstr(10, 2, "Current YOLO model:", curses.A_BOLD)
        param_win.addstr(10, 20, controller.current_yolo_model, curses.color_pair(2))
        
        # Help window
        help_win.clear()
        help_win.box()
        help_win.addstr(0, 2, "Controls", curses.A_BOLD)
        help_win.addstr(1, 2, "↑/↓: Select parameter")
        help_win.addstr(2, 2, "←/→: Change value")
        help_win.addstr(3, 2, "U: Update filter")
        help_win.addstr(4, 2, "S: Save preset")
        help_win.addstr(5, 2, "L: Load preset")
        help_win.addstr(6, 2, "Y: Switch YOLO model")
        help_win.addstr(7, 2, "R: Refresh YOLO models")
        help_win.addstr(8, 2, "Q: Quit")
        help_win.addstr(10, 2, "Status:", curses.A_BOLD)
        help_win.addstr(10, 10, status_message[:36])
        
        # YOLO models window
        yolo_win.clear()
        yolo_win.box()
        yolo_win.addstr(0, 2, "Available YOLO Models", curses.A_BOLD)
        if controller.yolo_models:
            for i, model in enumerate(controller.yolo_models[:5]):  # Show up to 5 models
                if model == controller.current_yolo_model:
                    yolo_win.addstr(i+1, 2, f"* {model}", curses.color_pair(2) | curses.A_BOLD)
                else:
                    yolo_win.addstr(i+1, 2, f"  {model}")
            if len(controller.yolo_models) > 5:
                yolo_win.addstr(6, 2, f"  ... and {len(controller.yolo_models) - 5} more")
        else:
            yolo_win.addstr(1, 2, "No models found in /yolo_models directory", curses.color_pair(3))
        
        # Status window
        status_win.clear()
        status_win.box()
        status_win.addstr(0, 2, "Command", curses.A_BOLD)
        status_win.addstr(1, 2, f"> {command}", curses.color_pair(2))
        
        # Input window
        input_win.clear()
        input_win.box()
        input_win.addstr(0, 2, "Input", curses.A_BOLD)
        input_win.addstr(1, 2, "> " + command)
        input_win.refresh()
        
        # Refresh all windows
        stdscr.refresh()
        param_win.refresh()
        help_win.refresh()
        status_win.refresh()
        yolo_win.refresh()
        
        # Get key
        key = stdscr.getch()
        
        # Process key
        if key == ord('q') or key == ord('Q'):
            break
        elif key == curses.KEY_UP:
            selected_param = (selected_param - 1) % len(param_names)
        elif key == curses.KEY_DOWN:
            selected_param = (selected_param + 1) % len(param_names)
        elif key == curses.KEY_LEFT or key == curses.KEY_RIGHT:
            # Adjust values
            delta = 0.05 if selected_param < 3 else 10
            delta = delta if key == curses.KEY_RIGHT else -delta
            
            if selected_param == 0:
                controller.tolerance = max(0, min(1, controller.tolerance + delta))
            elif selected_param == 1:
                controller.min_confidence = max(0, min(1, controller.min_confidence + delta))
            elif selected_param == 2:
                controller.min_area = max(0, min(1, controller.min_area + delta))
            elif selected_param >= 3:  # RGB values
                rgb_idx = selected_param - 3
                current = controller.rgb_range[rgb_idx]
                controller.rgb_range[rgb_idx] = max(0, min(255, int(current + delta)))
        elif key == ord('u') or key == ord('U'):
            result = controller.update_filter()
            status_message = "Parameters updated!" if result else "Failed to update parameters"
        elif key == ord('r') or key == ord('R'):
            controller.yolo_models = controller.get_available_yolo_models()
            status_message = f"Found {len(controller.yolo_models)} YOLO models"
        elif key == ord('s') or key == ord('S'):
            # Switch to input mode for saving
            command = "save "
            curses.curs_set(1)  # Show cursor
            input_win.clear()
            input_win.box()
            input_win.addstr(0, 2, "Save Preset", curses.A_BOLD)
            input_win.addstr(1, 2, "> " + command)
            input_win.refresh()
            
            # Get preset name
            curses.echo()
            preset_name = input_win.getstr(1, len(command) + 3).decode('utf-8')
            curses.noecho()
            curses.curs_set(0)  # Hide cursor
            
            if preset_name:
                controller.save_preset(preset_name)
                status_message = f"Saved preset: {preset_name}"
            command = ""
        elif key == ord('l') or key == ord('L'):
            # Switch to input mode for loading
            command = "load "
            curses.curs_set(1)  # Show cursor
            input_win.clear()
            input_win.box()
            input_win.addstr(0, 2, "Load Preset", curses.A_BOLD)
            input_win.addstr(1, 2, "> " + command)
            input_win.refresh()
            
            # Get preset name
            curses.echo()
            preset_name = input_win.getstr(1, len(command) + 3).decode('utf-8')
            curses.noecho()
            curses.curs_set(0)  # Hide cursor
            
            if preset_name:
                if controller.load_preset(preset_name):
                    controller.update_filter()
                    status_message = f"Loaded preset: {preset_name}"
                else:
                    status_message = f"Preset not found: {preset_name}"
            command = ""
        elif key == ord('y') or key == ord('Y'):
            # Switch to input mode for YOLO model
            command = "yolo "
            curses.curs_set(1)  # Show cursor
            input_win.clear()
            input_win.box()
            input_win.addstr(0, 2, "Switch YOLO Model", curses.A_BOLD)
            input_win.addstr(1, 2, "> " + command + " ")
            input_win.refresh()
            
            # Get model name
            curses.echo()
            model_name = input_win.getstr(1, len(command) + 3).decode('utf-8')
            curses.noecho()
            curses.curs_set(0)  # Hide cursor
            
            if model_name:
                if controller.switch_yolo_model(model_name):
                    status_message = f"Switched YOLO model: {model_name}"
                else:
                    status_message = f"Error switching model"
            command = ""

def main():
    controller = ColorFilterController()
    
    # Check for command line arguments
    if len(sys.argv) > 1:
        # Process command line arguments for non-interactive mode
        if sys.argv[1] == "load" and len(sys.argv) > 2:
            # Load preset and update filter
            preset_name = sys.argv[2]
            if controller.load_preset(preset_name):
                controller.update_filter()
                sys.exit(0)
            else:
                sys.exit(1)
        elif sys.argv[1] == "ui":
            # Run curses UI (advanced interface)
            try:
                curses.wrapper(run_curses_interface, controller)
            except Exception as e:
                print(f"Error in curses interface: {e}")
                controller.run_interactive()
        else:
            print("Unknown command line arguments")
            print("Usage: color_filter_controller.py [load preset_name | ui]")
    else:
        # Run interactive mode
        controller.run_interactive()

if __name__ == "__main__":
    main()