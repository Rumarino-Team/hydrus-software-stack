#!/usr/bin/env python3
import rospy
import os
import time
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detections
from std_msgs.msg import String
import math

class SlalomASCIIVisualizer:
    def __init__(self):
        rospy.init_node('slalom_ascii_visualizer', anonymous=True)
        
        # Display configuration
        self.width = 80
        self.height = 25
        self.scale_x = 2.0  # meters per character
        self.scale_y = 1.0  # meters per character
        self.origin_x = 10  # submarine start position
        self.origin_y = 5
        
        # Mission data
        self.submarine_pose = None
        self.current_detections = None
        self.gate_positions = []
        self.mission_status = {}
        self.gates_completed = []
        
        # Subscribers
        self.pose_sub = rospy.Subscriber('/zed2i/zed_node/pose', PoseStamped, self.pose_callback)
        self.detection_sub = rospy.Subscriber('/detector/box_detection', Detections, self.detection_callback)
        self.status_sub = rospy.Subscriber('/mission_status', String, self.status_callback)
        
        # Track submarine path
        self.submarine_trail = []
        self.max_trail_length = 20
        
        # Animation frame counter
        self.frame_count = 0
        
        rospy.loginfo("🎨 SlalomASCII Visualizer started!")
    
    def pose_callback(self, msg):
        """Update submarine position"""
        self.submarine_pose = msg
        
        # Add to trail
        pos = (msg.pose.position.x, msg.pose.position.y)
        self.submarine_trail.append(pos)
        if len(self.submarine_trail) > self.max_trail_length:
            self.submarine_trail.pop(0)
    
    def detection_callback(self, msg):
        """Update detections"""
        self.current_detections = msg
        
        # Extract gate positions from detections
        self.update_gate_positions_from_detections()
    
    def status_callback(self, msg):
        """Update mission status"""
        try:
            import json
            self.mission_status = json.loads(msg.data)
        except:
            pass
    
    def update_gate_positions_from_detections(self):
        """Extract gate positions from current detections"""
        if not self.current_detections:
            return
        
        # Find red and white pipes
        red_pipes = [d for d in self.current_detections.detections if d.cls == 2]
        white_pipes = [d for d in self.current_detections.detections if d.cls == 1]
        
        if len(red_pipes) >= 1 and len(white_pipes) >= 2:
            # Use closest red pipe as center
            red_pipe = min(red_pipes, key=lambda p: abs(p.point.x))
            center_y = red_pipe.point.y
            
            # Find white pipes on sides
            left_pipes = [p for p in white_pipes if p.point.y < center_y]
            right_pipes = [p for p in white_pipes if p.point.y > center_y]
            
            if left_pipes and right_pipes:
                left_pipe = max(left_pipes, key=lambda p: p.point.y)
                right_pipe = min(right_pipes, key=lambda p: p.point.y)
                
                # Store gate info
                gate_info = {
                    'center': red_pipe.point,
                    'left': left_pipe.point,
                    'right': right_pipe.point,
                    'target': Point(
                        x=(red_pipe.point.x + left_pipe.point.x) / 2,
                        y=(red_pipe.point.y + left_pipe.point.y) / 2,
                        z=red_pipe.point.z
                    )
                }
                
                # Update or add gate
                gate_found = False
                for i, existing_gate in enumerate(self.gate_positions):
                    # Check if this is an update to existing gate
                    distance = math.sqrt(
                        (existing_gate['center'].x - gate_info['center'].x)**2 +
                        (existing_gate['center'].y - gate_info['center'].y)**2
                    )
                    if distance < 3.0:  # Same gate if within 3m
                        self.gate_positions[i] = gate_info
                        gate_found = True
                        break
                
                if not gate_found:
                    self.gate_positions.append(gate_info)
    
    def world_to_screen(self, x, y):
        """Convert world coordinates to screen coordinates"""
        screen_x = int((x / self.scale_x) + self.origin_x)
        screen_y = int((y / self.scale_y) + self.origin_y)
        return screen_x, screen_y
    
    def create_frame(self):
        """Create ASCII frame"""
        # Initialize frame
        frame = [[' ' for _ in range(self.width)] for _ in range(self.height)]
        
        # Draw border
        for x in range(self.width):
            frame[0][x] = '─'
            frame[self.height-1][x] = '─'
        for y in range(self.height):
            frame[y][0] = '│'
            frame[y][self.width-1] = '│'
        
        # Corners
        frame[0][0] = '┌'
        frame[0][self.width-1] = '┐'
        frame[self.height-1][0] = '└'
        frame[self.height-1][self.width-1] = '┘'
        
        # Draw coordinate grid (every 5 units)
        for world_x in range(-20, 40, 5):
            screen_x, screen_y = self.world_to_screen(world_x, 0)
            if 0 < screen_x < self.width-1:
                for y in range(1, self.height-1):
                    if frame[y][screen_x] == ' ':
                        frame[y][screen_x] = '┊'
        
        for world_y in range(-10, 30, 5):
            screen_x, screen_y = self.world_to_screen(0, world_y)
            if 0 < screen_y < self.height-1:
                for x in range(1, self.width-1):
                    if frame[screen_y][x] == ' ':
                        frame[screen_y][x] = '┈'
        
        # Draw gates
        for i, gate in enumerate(self.gate_positions):
            # Gate number
            screen_x, screen_y = self.world_to_screen(gate['center'].x, gate['center'].y)
            if 0 < screen_x < self.width-1 and 0 < screen_y < self.height-1:
                frame[screen_y][screen_x] = str(i+1)  # Gate number
            
            # Red pipe (center)
            screen_x, screen_y = self.world_to_screen(gate['center'].x, gate['center'].y)
            if 0 < screen_x < self.width-1 and 0 < screen_y < self.height-1:
                frame[screen_y][screen_x] = '●'  # Red pipe
            
            # White pipes
            screen_x, screen_y = self.world_to_screen(gate['left'].x, gate['left'].y)
            if 0 < screen_x < self.width-1 and 0 < screen_y < self.height-1:
                frame[screen_y][screen_x] = '○'  # White pipe left
            
            screen_x, screen_y = self.world_to_screen(gate['right'].x, gate['right'].y)
            if 0 < screen_x < self.width-1 and 0 < screen_y < self.height-1:
                frame[screen_y][screen_x] = '○'  # White pipe right
            
            # Target point
            screen_x, screen_y = self.world_to_screen(gate['target'].x, gate['target'].y)
            if 0 < screen_x < self.width-1 and 0 < screen_y < self.height-1:
                frame[screen_y][screen_x] = '×'  # Target
            
            # Gate connection line
            left_sx, left_sy = self.world_to_screen(gate['left'].x, gate['left'].y)
            right_sx, right_sy = self.world_to_screen(gate['right'].x, gate['right'].y)
            if left_sy == right_sy and 0 < left_sy < self.height-1:
                for x in range(min(left_sx, right_sx), max(left_sx, right_sx)+1):
                    if 0 < x < self.width-1 and frame[left_sy][x] == ' ':
                        frame[left_sy][x] = '─'
        
        # Draw submarine trail
        for i, (trail_x, trail_y) in enumerate(self.submarine_trail[:-1]):
            screen_x, screen_y = self.world_to_screen(trail_x, trail_y)
            if 0 < screen_x < self.width-1 and 0 < screen_y < self.height-1:
                alpha = i / len(self.submarine_trail)
                if alpha < 0.3:
                    char = '·'
                elif alpha < 0.6:
                    char = '•'
                else:
                    char = '▪'
                frame[screen_y][screen_x] = char
        
        # Draw submarine
        if self.submarine_pose:
            screen_x, screen_y = self.world_to_screen(
                self.submarine_pose.pose.position.x,
                self.submarine_pose.pose.position.y
            )
            if 0 < screen_x < self.width-1 and 0 < screen_y < self.height-1:
                # Animated submarine
                frame[screen_y][screen_x] = '▲'  # Simple submarine symbol
        
        return frame
    
    def create_status_panel(self):
        """Create status information panel"""
        lines = []
        lines.append("🌊 SLALOM MISSION STATUS")
        lines.append("=" * 50)
        
        if self.mission_status:
            lines.append(f"Gates passed: {self.mission_status.get('gates_passed', 0)}/{self.mission_status.get('total_gates', 3)}")
            lines.append(f"Completion: {self.mission_status.get('completion_percentage', 0):.1f}%")
            lines.append(f"Current gate: {self.mission_status.get('current_gate', 0) + 1}")
        
        if self.submarine_pose:
            pos = self.submarine_pose.pose.position
            lines.append(f"Position: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
        
        lines.append(f"Gates detected: {len(self.gate_positions)}")
        
        if self.current_detections:
            red_count = sum(1 for d in self.current_detections.detections if d.cls == 2)
            white_count = sum(1 for d in self.current_detections.detections if d.cls == 1)
            lines.append(f"Current detections: {red_count}R, {white_count}W")
        
        lines.append("")
        lines.append("LEGEND:")
        lines.append("▲ = Submarine    ● = Red pipe")
        lines.append("• = Trail        ○ = White pipe")
        lines.append("× = Target       ─ = Gate")
        lines.append("┊┈ = Grid        1,2,3 = Gate numbers")
        
        return lines
    
    def render_frame(self, frame, status_lines):
        """Render complete frame with status"""
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Print frame
        for row in frame:
            print(''.join(row))
        
        # Print status
        print()
        for line in status_lines:
            print(line)
        
        # Print navigation info
        print(f"\nFrame: {self.frame_count:04d} | Time: {rospy.Time.now().to_sec():.1f}")
    
    def run(self):
        """Main visualization loop"""
        rate = rospy.Rate(5)  # 5 FPS
        
        while not rospy.is_shutdown():
            try:
                # Create and render frame
                frame = self.create_frame()
                status = self.create_status_panel()
                self.render_frame(frame, status)
                
                self.frame_count += 1
                rate.sleep()
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"Visualization error: {e}")
                time.sleep(1)

if __name__ == '__main__':
    try:
        visualizer = SlalomASCIIVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()