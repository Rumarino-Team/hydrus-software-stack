#!/usr/bin/env python3
"""
Interactive Mission Controller for the Mission Planner (Terminal Version)

This script provides a terminal interface to interact with the mission manager,
allowing users to:
- Start, stop, and reset missions
- Select different missions
- View mission status with color-coded information

Author: GitHub Copilot
Date: May 18, 2025
"""

import rospy
import json
import curses
import threading
import time
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

# Color pairs for terminal display (will be initialized later)
COLOR_ACTIVE = 1      # Green
COLOR_INACTIVE = 2    # Red
COLOR_NORMAL = 3      # Blue
COLOR_WARNING = 4     # Yellow/Orange
COLOR_NEUTRAL = 5     # Grey
COLOR_HEADER = 6      # White on Blue
COLOR_PROGRESS = 7    # Green on Black

class MissionController:
    """Interactive terminal controller for the mission planner"""
    
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.setup_colors()
        
        # Initialize screen settings
        curses.curs_set(0)  # Hide cursor
        self.stdscr.clear()
        self.stdscr.refresh()
        
        # Get terminal dimensions
        self.height, self.width = self.stdscr.getmaxyx()
        
        # Initialize ROS node
        rospy.init_node('mission_controller', anonymous=True)
        
        # Mission status data
        self.mission_status = {}
        self.active_mission = None
        self.available_missions = []
        self.selected_index = 0
        self.current_view = 'main'  # 'main' or 'details'
        
        # Command mapping
        self.commands = {
            'main': {
                'q': self.quit,
                'KEY_UP': self.select_prev_mission,
                'KEY_DOWN': self.select_next_mission,
                'KEY_ENTER': self.select_mission,
                '\n': self.select_mission,
                's': self.start_mission,
                'x': self.stop_mission,
                'r': self.reset_mission,
                'd': lambda: self.set_view('details'),
            },
            'details': {
                'q': lambda: self.set_view('main'),
                'KEY_UP': self.scroll_details_up,
                'KEY_DOWN': self.scroll_details_down,
                'b': lambda: self.set_view('main'),
            }
        }
        
        # Details view scroll position
        self.details_scroll = 0
        self.details_content = []
        
        # Create windows
        self.setup_windows()
        
        # Connect to ROS services and topics
        self.connect_to_ros()
        
        # Start the ROS update thread
        self.running = True
        self.update_thread = threading.Thread(target=self.ros_update_thread)
        self.update_thread.daemon = True
        self.update_thread.start()
        
    def setup_colors(self):
        """Initialize color pairs"""
        curses.start_color()
        curses.use_default_colors()
        
        # Initialize color pairs
        curses.init_pair(COLOR_ACTIVE, curses.COLOR_GREEN, -1)
        curses.init_pair(COLOR_INACTIVE, curses.COLOR_RED, -1)
        curses.init_pair(COLOR_NORMAL, curses.COLOR_CYAN, -1)
        curses.init_pair(COLOR_WARNING, curses.COLOR_YELLOW, -1)
        curses.init_pair(COLOR_NEUTRAL, curses.COLOR_WHITE, -1)
        curses.init_pair(COLOR_HEADER, curses.COLOR_WHITE, curses.COLOR_BLUE)
        curses.init_pair(COLOR_PROGRESS, curses.COLOR_GREEN, curses.COLOR_BLACK)
        
    def setup_windows(self):
        """Create and setup all windows"""
        # Header window - top of screen
        header_height = 3
        self.header_win = curses.newwin(header_height, self.width, 0, 0)
        
        # Command bar - bottom of screen
        cmd_height = 2
        self.cmd_win = curses.newwin(cmd_height, self.width, self.height - cmd_height, 0)
        
        # Main content area
        content_height = self.height - header_height - cmd_height
        content_width = self.width
        
        # List window - left side of content area
        list_width = 30
        self.list_win = curses.newwin(content_height, list_width, header_height, 0)
        
        # Status window - right side of content area
        status_width = content_width - list_width
        status_x = list_width
        self.status_win = curses.newwin(content_height, status_width, header_height, status_x)
        
        # Details window (covers the whole content area when showing details)
        self.details_win = curses.newwin(content_height, content_width, header_height, 0)
        
    def connect_to_ros(self):
        """Connect to ROS services and topics"""
        # Subscribe to mission status topic
        rospy.Subscriber('/mission_manager/status', String, self.status_callback)
        
        # Connect to mission manager services
        try:
            rospy.wait_for_service('/mission_manager/start', timeout=2.0)
            self.start_service = rospy.ServiceProxy('/mission_manager/start', Trigger)
            
            rospy.wait_for_service('/mission_manager/stop', timeout=2.0)
            self.stop_service = rospy.ServiceProxy('/mission_manager/stop', Trigger)
            
            rospy.wait_for_service('/mission_manager/reset', timeout=2.0)
            self.reset_service = rospy.ServiceProxy('/mission_manager/reset', Trigger)
            
            rospy.wait_for_service('/mission_manager/select_mission', timeout=2.0)
            self.select_service = rospy.ServiceProxy('/mission_manager/select_mission', SetBool)
            
            rospy.loginfo("Successfully connected to all mission manager services")
            self.show_message("Connected to mission manager services")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to mission manager services: {e}")
            self.show_message(f"ERROR: Failed to connect to mission manager services: {e}")
    
    def status_callback(self, msg):
        """Callback for status messages"""
        try:
            status_dict = json.loads(msg.data)
            self.mission_status = status_dict.get("missions", {})
            self.active_mission = status_dict.get("active_mission", None)
            
            # Update the available missions list if needed
            if list(self.mission_status.keys()) != self.available_missions:
                self.available_missions = list(self.mission_status.keys())
                # Adjust selected index if it's now out of bounds
                if self.available_missions and self.selected_index >= len(self.available_missions):
                    self.selected_index = len(self.available_missions) - 1
                
            # Generate details content
            self.generate_details_content()
                
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to decode mission status: {e}")
    
    def generate_details_content(self):
        """Generate the content for details view"""
        self.details_content = []
        
        if not self.mission_status:
            self.details_content.append(("No mission data available.", COLOR_WARNING))
            return
            
        # Header
        self.details_content.append(("MISSION PLANNER STATUS", COLOR_HEADER))
        self.details_content.append(("-" * 50, COLOR_NEUTRAL))
        self.details_content.append((f"Active Mission: {self.active_mission or 'None'}", 
                                   COLOR_ACTIVE if self.active_mission else COLOR_WARNING))
        self.details_content.append(("", COLOR_NEUTRAL))  # Empty line
        
        # Missions list
        self.details_content.append(("AVAILABLE MISSIONS", COLOR_HEADER))
        self.details_content.append(("-" * 50, COLOR_NEUTRAL))
        
        for mission_name, mission_data in self.mission_status.items():
            is_active = mission_name == self.active_mission
            name_color = COLOR_ACTIVE if is_active else COLOR_NEUTRAL
            
            self.details_content.append((f"{mission_name}:", name_color))
            
            mission_active = mission_data.get("active", False)
            active_text = "ACTIVE" if mission_active else "INACTIVE"
            active_color = COLOR_ACTIVE if mission_active else COLOR_INACTIVE
            self.details_content.append((f"  Status: {active_text}", active_color))
            
            # Display mission details
            for key, value in mission_data.items():
                if key != "active":
                    # Determine color based on value type
                    if isinstance(value, bool):
                        color = COLOR_ACTIVE if value else COLOR_INACTIVE
                    elif isinstance(value, (int, float)) and "percentage" in key:
                        color = (COLOR_ACTIVE if value > 60 else 
                               COLOR_WARNING if value > 30 else COLOR_INACTIVE)
                    else:
                        color = COLOR_NEUTRAL
                    
                    self.details_content.append((f"  {key}: {value}", color))
            
            self.details_content.append(("", COLOR_NEUTRAL))  # Empty line
            
    def draw_progress_bar(self, win, y, x, width, value, max_value=100):
        """Draw a progress bar"""
        win.addstr(y, x, "[", curses.color_pair(COLOR_NEUTRAL))
        
        bar_width = width - 2
        filled_width = int(bar_width * value / max_value)
        
        for i in range(bar_width):
            if i < filled_width:
                win.addstr(" ", curses.color_pair(COLOR_PROGRESS) | curses.A_REVERSE)
            else:
                win.addstr(" ", curses.color_pair(COLOR_NEUTRAL))
                
        win.addstr("]", curses.color_pair(COLOR_NEUTRAL))
    
    def draw_header(self):
        """Draw the header window"""
        self.header_win.clear()
        self.header_win.bkgd(' ', curses.color_pair(COLOR_HEADER))
        
        title = "MISSION PLANNER CONTROL CENTER"
        x = max(0, (self.width - len(title)) // 2)
        self.header_win.addstr(1, x, title, curses.A_BOLD)
        
        self.header_win.refresh()
    
    def draw_command_bar(self):
        """Draw the command bar at the bottom"""
        self.cmd_win.clear()
        
        if self.current_view == 'main':
            commands = "[↑/↓] Navigate  [Enter] Select  [s] Start  [x] Stop  [r] Reset  [d] Details  [q] Quit"
        else:  # details view
            commands = "[↑/↓] Scroll  [b] Back  [q] Quit"
            
        self.cmd_win.addstr(0, 0, commands, curses.color_pair(COLOR_NEUTRAL))
        self.cmd_win.refresh()
    
    def draw_mission_list(self):
        """Draw the mission list window"""
        self.list_win.clear()
        self.list_win.box()
        
        title = " Mission Selection "
        self.list_win.addstr(0, 2, title, curses.A_BOLD)
        
        if not self.available_missions:
            self.list_win.addstr(2, 2, "No missions available", curses.color_pair(COLOR_WARNING))
        else:
            for i, mission in enumerate(self.available_missions):
                # Highlight if selected
                if i == self.selected_index:
                    attr = curses.A_REVERSE
                else:
                    attr = curses.A_NORMAL
                
                # Mark active mission with an asterisk
                if mission == self.active_mission:
                    mission_text = f"* {mission}"
                    color = COLOR_ACTIVE
                else:
                    mission_text = f"  {mission}"
                    color = COLOR_NEUTRAL
                
                # Ensure we don't write outside the window
                if 2 + i < self.list_win.getmaxyx()[0] - 1:
                    self.list_win.addstr(2 + i, 1, mission_text, curses.color_pair(color) | attr)
        
        self.list_win.refresh()
    
    def draw_status(self):
        """Draw the status window"""
        self.status_win.clear()
        self.status_win.box()
        
        title = " Mission Status "
        self.status_win.addstr(0, 2, title, curses.A_BOLD)
        
        # Draw active mission and status
        line = 2
        self.status_win.addstr(line, 2, "Active Mission:", curses.color_pair(COLOR_NEUTRAL))
        line += 1
        
        if self.active_mission:
            self.status_win.addstr(line, 4, self.active_mission, curses.color_pair(COLOR_ACTIVE))
        else:
            self.status_win.addstr(line, 4, "None", curses.color_pair(COLOR_WARNING))
        line += 2
        
        # Show status for active mission
        if self.active_mission and self.active_mission in self.mission_status:
            mission_data = self.mission_status[self.active_mission]
            
            # Active/Inactive status
            is_active = mission_data.get("active", False)
            status_text = "Active" if is_active else "Inactive"
            status_color = COLOR_ACTIVE if is_active else COLOR_INACTIVE
            
            self.status_win.addstr(line, 2, "Status:", curses.color_pair(COLOR_NEUTRAL))
            self.status_win.addstr(line, 10, status_text, curses.color_pair(status_color))
            line += 2
            
            # Show progress if available
            if "completion_percentage" in mission_data:
                percentage = mission_data["completion_percentage"]
                self.status_win.addstr(line, 2, "Progress:", curses.color_pair(COLOR_NEUTRAL))
                line += 1
                
                # Draw a progress bar
                bar_width = self.status_win.getmaxyx()[1] - 6
                self.draw_progress_bar(self.status_win, line, 2, bar_width, percentage)
                line += 1
                
                # Show percentage
                self.status_win.addstr(line, 2, f"{percentage:.1f}%", 
                                     curses.color_pair(COLOR_NEUTRAL))
                line += 2
            
            # Show other mission data
            self.status_win.addstr(line, 2, "Mission Data:", curses.color_pair(COLOR_NORMAL))
            line += 1
            
            for key, value in mission_data.items():
                if key not in ["active", "completion_percentage"]:
                    # Skip if we're running out of space
                    if line >= self.status_win.getmaxyx()[0] - 2:
                        break
                    
                    # Format the value properly
                    value_str = f"{value}"
                    if isinstance(value, bool):
                        color = COLOR_ACTIVE if value else COLOR_INACTIVE
                    elif isinstance(value, (int, float)):
                        color = COLOR_NEUTRAL
                    else:
                        color = COLOR_NEUTRAL
                    
                    self.status_win.addstr(line, 4, f"{key}: {value_str}", 
                                         curses.color_pair(color))
                    line += 1
        else:
            self.status_win.addstr(line, 2, "No active mission selected", 
                                 curses.color_pair(COLOR_WARNING))
        
        self.status_win.refresh()
    
    def draw_details(self):
        """Draw the details window"""
        self.details_win.clear()
        self.details_win.box()
        
        title = " Mission Details "
        self.details_win.addstr(0, 2, title, curses.A_BOLD)
        
        max_lines = self.details_win.getmaxyx()[0] - 2  # Account for box
        
        for i, (text, color) in enumerate(self.details_content[self.details_scroll:]):
            if i >= max_lines:
                break
                
            self.details_win.addstr(i + 1, 2, text, curses.color_pair(color))
        
        # Show scroll indicators if needed
        if self.details_scroll > 0:
            self.details_win.addstr(1, self.details_win.getmaxyx()[1] - 3, "↑", 
                                  curses.color_pair(COLOR_NORMAL))
                                  
        if self.details_scroll + max_lines < len(self.details_content):
            self.details_win.addstr(max_lines, self.details_win.getmaxyx()[1] - 3, "↓", 
                                  curses.color_pair(COLOR_NORMAL))
        
        self.details_win.refresh()
    
    def draw_screen(self):
        """Draw all windows based on current view"""
        # Always draw header and command bar
        self.draw_header()
        self.draw_command_bar()
        
        # Draw content based on current view
        if self.current_view == 'main':
            self.draw_mission_list()
            self.draw_status()
        else:  # details view
            self.draw_details()
    
    def show_message(self, message, delay=1.5):
        """Show a message in the center of the screen"""
        h, w = self.stdscr.getmaxyx()
        y = h // 2
        x = max(0, (w - len(message)) // 2)
        
        # Create a popup window
        msg_h, msg_w = 3, len(message) + 4
        msg_win = curses.newwin(msg_h, msg_w, max(0, y - 1), max(0, x - 2))
        msg_win.clear()
        msg_win.box()
        msg_win.addstr(1, 2, message)
        msg_win.refresh()
        
        # Wait before removing
        curses.napms(int(delay * 1000))
        
        # Force redraw
        self.draw_screen()
    
    def set_view(self, view):
        """Change the current view"""
        self.current_view = view
        if view == 'details':
            self.details_scroll = 0  # Reset scroll position
        self.draw_screen()
    
    def select_next_mission(self):
        """Select the next mission in the list"""
        if self.available_missions:
            self.selected_index = (self.selected_index + 1) % len(self.available_missions)
            self.draw_mission_list()
    
    def select_prev_mission(self):
        """Select the previous mission in the list"""
        if self.available_missions:
            self.selected_index = (self.selected_index - 1) % len(self.available_missions)
            self.draw_mission_list()
    
    def scroll_details_up(self):
        """Scroll details view up"""
        if self.details_scroll > 0:
            self.details_scroll -= 1
            self.draw_details()
    
    def scroll_details_down(self):
        """Scroll details view down"""
        max_lines = self.details_win.getmaxyx()[0] - 2  # Account for box
        if self.details_scroll + max_lines < len(self.details_content):
            self.details_scroll += 1
            self.draw_details()
    
    def select_mission(self):
        """Select the currently highlighted mission"""
        if not self.available_missions:
            self.show_message("No missions available")
            return
            
        selected_mission = self.available_missions[self.selected_index]
        
        try:
            # Service expects a boolean parameter but we'll use a string in data field
            response = self.select_service(selected_mission)
            if response.success:
                self.show_message(f"Selected mission: {selected_mission}")
            else:
                self.show_message(f"ERROR: {response.message}")
        except rospy.ServiceException as e:
            self.show_message(f"Service error: {e}")
    
    def start_mission(self):
        """Start the active mission"""
        try:
            response = self.start_service()
            if response.success:
                self.show_message("Mission started!")
            else:
                self.show_message(f"ERROR: {response.message}")
        except rospy.ServiceException as e:
            self.show_message(f"Service error: {e}")
    
    def stop_mission(self):
        """Stop the active mission"""
        try:
            response = self.stop_service()
            if response.success:
                self.show_message("Mission stopped")
            else:
                self.show_message(f"ERROR: {response.message}")
        except rospy.ServiceException as e:
            self.show_message(f"Service error: {e}")
    
    def reset_mission(self):
        """Reset the active mission"""
        try:
            response = self.reset_service()
            if response.success:
                self.show_message("Mission reset")
            else:
                self.show_message(f"ERROR: {response.message}")
        except rospy.ServiceException as e:
            self.show_message(f"Service error: {e}")
    
    def quit(self):
        """Quit the application"""
        self.running = False
        return False  # Return False to exit the main loop
    
    def ros_update_thread(self):
        """Background thread to handle ROS updates"""
        rate = rospy.Rate(10)  # 10Hz
        while self.running and not rospy.is_shutdown():
            try:
                # Queue a screen redraw (thread-safe)
                curses.ungetch(curses.KEY_RESIZE)
                rate.sleep()
            except rospy.ROSInterruptException:
                break
    
    def run(self):
        """Main run loop"""
        self.draw_screen()
        
        while True:
            try:
                # Get key input (with timeout)
                self.stdscr.timeout(100)
                key = self.stdscr.getkey()
                
                # Check for resize events
                if key == "KEY_RESIZE":
                    # Update dimensions
                    self.height, self.width = self.stdscr.getmaxyx()
                    self.setup_windows()
                    self.draw_screen()
                    continue
                
                # Handle commands based on current view
                if key in self.commands[self.current_view]:
                    result = self.commands[self.current_view][key]()
                    if result is False:  # Special case for quit
                        break
                    self.draw_screen()
                    
            except curses.error:
                # No input available or other curses error
                pass
                
            except KeyboardInterrupt:
                break

def main(stdscr):
    # Initialize controller with the main screen
    controller = MissionController(stdscr)
    controller.run()

if __name__ == "__main__":
    try:
        # Initialize curses and run main
        curses.wrapper(main)
    except Exception as e:
        # Make sure terminal is usable even on error
        if 'stdscr' in locals():
            curses.nocbreak()
            curses.echo()
            curses.endwin()
        print(f"Error: {e}")
    except rospy.ROSInterruptException:
        pass