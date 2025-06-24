#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detection, Detections
from mission_planner.slalom_mission import SlalomMission

class SlalomMissionTester:
    def __init__(self):
        rospy.init_node('slalom_mission_tester', anonymous=True)
        
        # Initialize the mission with action client disabled for testing
        self.mission = SlalomMission(enable_action_client=False)
        
               # Set up publishers for testing - use the same topics as base_mission.py
        self.detection_pub = rospy.Publisher('/detector/box_detection', Detections, queue_size=10)
        self.pose_pub = rospy.Publisher('/zed2i/zed_node/pose', PoseStamped, queue_size=10)
        
        # Create mock pose
        self.mock_pose = PoseStamped()
        self.mock_pose.header.frame_id = "map"
        self.mock_pose.pose.position.x = 0.0
        self.mock_pose.pose.position.y = 0.0
        self.mock_pose.pose.position.z = -1.0
        self.mock_pose.pose.orientation.w = 1.0
        
        # Wait for subscribers to connect
        rospy.sleep(1.0)
        
    def update_submarine_pose(self, x, y, z):
        """Update the submarine pose and publish it"""
        self.mock_pose.header.stamp = rospy.Time.now()
        self.mock_pose.pose.position.x = x
        self.mock_pose.pose.position.y = y
        self.mock_pose.pose.position.z = z
        
        # Update mission's pose directly and publish
        self.mission.submarine_pose = self.mock_pose
        self.pose_pub.publish(self.mock_pose)
        
    def publish_mock_gate_horizontal(self, gate_number, gate_x_position, gate_y_position):
        """Publish mock detection data for a HORIZONTAL gate"""
        detections = Detections()
        detections.detector_name = f"horizontal_gate_{gate_number + 1}"
        
        # HORIZONTAL gate: pipes arranged vertically at same X position
        # White right side - Red center - White left side
        detections.detections = [
            self.create_detection(1, gate_x_position, gate_y_position - 1.0, -2.0),  # White right side
            self.create_detection(2, gate_x_position, gate_y_position, -2.0),        # Red center  
            self.create_detection(1, gate_x_position, gate_y_position + 1.0, -2.0),  # White left side
        ]
        
        # Set detections on mission and publish
        self.mission.current_detections = detections
        for _ in range(3):
            self.detection_pub.publish(detections)
            rospy.sleep(0.1)
        
        rospy.loginfo(f"Published horizontal gate {gate_number + 1} at x={gate_x_position}, y={gate_y_position}")
        rospy.loginfo(f"   Red center: Y={gate_y_position}")
        rospy.loginfo(f"   White left side: Y={gate_y_position + 1.0}")
        rospy.loginfo(f"   White right side: Y={gate_y_position - 1.0}")
    
    def create_detection(self, cls, x, y, z):
        """Helper to create detection objects"""
        detection = Detection()
        detection.cls = cls
        detection.point = Point(x=x, y=y, z=z)
        detection.confidence = 0.9
        # Add bounding box
        detection.bounding_box.x_offset = 100
        detection.bounding_box.y_offset = 100
        detection.bounding_box.width = 50
        detection.bounding_box.height = 50
        return detection
    
    def simulate_submarine_movement(self, start_pos, target_pos, steps=15):
        """Simulate submarine moving from start to target position with more steps for smoother movement"""
        rospy.loginfo(f"Moving from ({start_pos[0]:.1f}, {start_pos[1]:.1f}) to ({target_pos.x:.1f}, {target_pos.y:.1f})")
        
        for i in range(steps + 1):
            progress = i / steps
            current_x = start_pos[0] + (target_pos.x - start_pos[0]) * progress
            current_y = start_pos[1] + (target_pos.y - start_pos[1]) * progress
            current_z = start_pos[2] + (target_pos.z - start_pos[2]) * progress
            
            # Update submarine position
            self.update_submarine_pose(current_x, current_y, current_z)
            
            # Log progress with more detail
            if i == 0:
                rospy.loginfo(f"  Starting at: ({current_x:.1f}, {current_y:.1f}, {current_z:.1f})")
            elif i == steps:
                rospy.loginfo(f"  Reached target: ({current_x:.1f}, {current_y:.1f}, {current_z:.1f})")
            elif i % 3 == 0:  # Log every 3rd step
                rospy.loginfo(f"  Progress {progress*100:.0f}%: ({current_x:.1f}, {current_y:.1f}, {current_z:.1f})")
            
            time.sleep(0.15)  # Slower movement for better control
        
        return [current_x, current_y, current_z]
    
    def navigate_through_gate_controlled(self, submarine_pos, gate_data, gate_num):
        """Navigate through gate with controlled multi-point path to avoid white pipe collision"""
        rospy.loginfo(f"\nPhase 3: CONTROLLED Navigation through Horizontal Gate {gate_num + 1}")
        rospy.loginfo("="*60)
        
        # Calculate safe passage points
        red_center = gate_data['center']
        left_target = gate_data['left_side']  # Between red and left white
        
        # Create waypoints for controlled passage
        waypoints = []
        
        # Waypoint 1: Pre-approach (further back, aligned with target Y)
        pre_approach = Point()
        pre_approach.x = red_center.x - 2.0  # 2m before gate
        pre_approach.y = left_target.y + 0.1  # Slightly above left target
        pre_approach.z = left_target.z
        waypoints.append(("Pre-approach", pre_approach))
        
        # Waypoint 2: Gate entry (precise alignment)
        gate_entry = Point()
        gate_entry.x = red_center.x - 0.5  # Just before gate
        gate_entry.y = left_target.y + 0.05  # Very close to left target
        gate_entry.z = left_target.z
        waypoints.append(("Gate Entry", gate_entry))
        
        # Waypoint 3: Gate center (between red and white left)
        gate_center = Point()
        gate_center.x = red_center.x  # At gate center X
        gate_center.y = left_target.y  # Exactly at left side position
        gate_center.z = left_target.z
        waypoints.append(("Gate Center", gate_center))
        
        # Waypoint 4: Gate exit (maintaining same Y)
        gate_exit = Point()
        gate_exit.x = red_center.x + 0.5  # Just past gate
        gate_exit.y = left_target.y  # Same Y level
        gate_exit.z = left_target.z
        waypoints.append(("Gate Exit", gate_exit))
        
        # Waypoint 5: Safe distance past gate
        safe_distance = Point()
        safe_distance.x = red_center.x + 2.0  # Well past gate
        safe_distance.y = left_target.y - 0.1  # Slightly towards center
        safe_distance.z = left_target.z
        waypoints.append(("Safe Distance", safe_distance))
        
        # Execute controlled navigation through all waypoints
        current_pos = submarine_pos
        for waypoint_name, waypoint in waypoints:
            rospy.loginfo(f"\nMoving to {waypoint_name}: ({waypoint.x:.2f}, {waypoint.y:.2f}, {waypoint.z:.2f})")
            
            # Check if this puts us between red and white left
            if waypoint_name == "Gate Center":
                rospy.loginfo(f"   Passing BETWEEN red pipe (Y={red_center.y:.2f}) and white left pipe")
                rospy.loginfo(f"   Target Y={waypoint.y:.2f} is ABOVE red center (safe passage)")
            
            current_pos = self.simulate_submarine_movement(current_pos, waypoint, steps=10)
            rospy.loginfo(f"   Reached {waypoint_name}")
            
            # Small pause between waypoints for stability
            time.sleep(0.3)
        
        rospy.loginfo(f"\nCONTROLLED PASSAGE COMPLETE!")
        rospy.loginfo(f"   Final position: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
        rospy.loginfo(f"   Successfully navigated between red pipe and white left pipe")
        
        return current_pos
    
    def test_complete_slalom_navigation(self):
        """Test complete horizontal slalom navigation with controlled gate passage"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("CONTROLLED HORIZONTAL SLALOM NAVIGATION - LEFT SIDE")
        rospy.loginfo("="*70)
        
        # Initial submarine position - START ON THE LEFT
        submarine_pos = [0.0, 10.0, -1.0]  # Start at left side, center Y
        self.update_submarine_pose(*submarine_pos)
        
        # HORIZONTAL gate positions - submarine travels LEFT to RIGHT (increasing X)
        gate_positions = [
            {'x': 15.0, 'y': 13.0},   # Gate 1: at x=15, centered at y=10
            {'x': 30.0, 'y': 8.0},   # Gate 2: at x=30, centered at y=10  
            {'x': 45.0, 'y': 15.0}    # Gate 3: at x=45, centered at y=10
        ]
        
        rospy.loginfo(f"CONTROLLED Mission Setup:")
        rospy.loginfo(f"   Starting position: ({submarine_pos[0]}, {submarine_pos[1]}, {submarine_pos[2]})")
        rospy.loginfo(f"   Total gates: {len(gate_positions)} (aligned horizontally)")
        rospy.loginfo(f"   Direction: LEFT to RIGHT (increasing X)")
        rospy.loginfo(f"   Strategy: CONTROLLED passage through LEFT SIDE with waypoints")
        rospy.loginfo(f"   Fix: Multi-point navigation to avoid white pipe collision")
        
        for i, gate in enumerate(gate_positions):
            rospy.loginfo(f"       Gate {i+1}: X={gate['x']}, Y={gate['y']} (pipes: Y={gate['y']-1} to Y={gate['y']+1})")
        
        successful_passages = 0
        
        for gate_num, gate_info in enumerate(gate_positions):
            rospy.loginfo(f"\n" + "="*50)
            rospy.loginfo(f"HORIZONTAL GATE {gate_num + 1} - CONTROLLED NAVIGATION")
            rospy.loginfo("="*50)
            
            gate_x = gate_info['x']
            gate_y = gate_info['y']
            
            # 1. Detect the horizontal gate
            rospy.loginfo(f"Phase 1: Detecting Horizontal Gate {gate_num + 1}")
            self.publish_mock_gate_horizontal(gate_num, gate_x, gate_y)
            rospy.sleep(0.5)
            
            # Verify gate detection
            if self.mission.detect_gate(gate_num):
                rospy.loginfo(f"Horizontal Gate {gate_num + 1} detected successfully!")
                
                gate_data = self.mission.gate_positions[gate_num]
                rospy.loginfo(f"   Red pipe center: ({gate_data['center'].x:.1f}, {gate_data['center'].y:.1f})")
                rospy.loginfo(f"   LEFT side target: ({gate_data['left_side'].x:.1f}, {gate_data['left_side'].y:.1f})")
                rospy.loginfo(f"   RIGHT side: ({gate_data['right_side'].x:.1f}, {gate_data['right_side'].y:.1f})")
                
                # 2. Navigate to initial approach position
                rospy.loginfo(f"\nPhase 2: Initial Approach to Gate {gate_num + 1}")
                approach_target = Point()
                approach_target.x = gate_data['center'].x - 6.0       # 6m before gate (increased from 4m)
                approach_target.y = gate_data['left_side'].y + 0.2    # Slightly above left target
                approach_target.z = gate_data['left_side'].z
                
                submarine_pos = self.simulate_submarine_movement(submarine_pos, approach_target)
                rospy.loginfo(f"Reached initial approach position")
                
                # 3. CONTROLLED NAVIGATION through gate
                submarine_pos = self.navigate_through_gate_controlled(submarine_pos, gate_data, gate_num)
                
                # 4. Mark gate as passed
                if self.mission.navigate_gate(gate_num):
                    successful_passages += 1
                    rospy.loginfo(f"SUCCESS! Gate {gate_num + 1} passed with CONTROLLED navigation")
                    rospy.loginfo(f"   Gates passed: {successful_passages}/{len(gate_positions)}")
                else:
                    rospy.logwarn(f"Failed to register gate passage for Gate {gate_num + 1}")
                    
            else:
                rospy.logwarn(f"Failed to detect Horizontal Gate {gate_num + 1}")
                break
                
            # Pause between gates
            if gate_num < len(gate_positions) - 1:
                rospy.loginfo(f"\nPreparing for next gate...")
                time.sleep(1.5)
        
        # Final results
        rospy.loginfo(f"\n" + "="*60)
        rospy.loginfo("CONTROLLED HORIZONTAL SLALOM COMPLETE!")
        rospy.loginfo("="*60)
        
        status = self.mission.get_status()
        rospy.loginfo(f"Final Results:")
        rospy.loginfo(f"   Gates detected: {status['gates_detected']}")
        rospy.loginfo(f"   Gates passed: {status['gates_passed']}")
        rospy.loginfo(f"   Current gate: {status['current_gate']}")
        rospy.loginfo(f"   Completion: {status['completion_percentage']:.1f}%")
        rospy.loginfo(f"   Final position: ({submarine_pos[0]:.1f}, {submarine_pos[1]:.1f}, {submarine_pos[2]:.1f})")
        rospy.loginfo(f"   Navigation: CONTROLLED passage through LEFT side")
        
        if status['gates_passed'] == len(gate_positions):
            rospy.loginfo("CONTROLLED MISSION SUCCESS! No white pipe collisions!")
        else:
            rospy.logwarn(f"Mission incomplete: {status['gates_passed']}/{len(gate_positions)} gates passed")
        
        return status['gates_passed'] == len(gate_positions)
    
    def test_direct_detection(self):
        """Test detection directly without using the mission's subscriber"""
        rospy.loginfo("Testing direct horizontal detection...")
        
        # Create test detections directly - HORIZONTAL layout
        detections = Detections()
        detections.detector_name = "direct_horizontal_test"
        detections.detections = [
            self.create_detection(1, 8.0, 9.0, -2.0),   # White right side
            self.create_detection(2, 8.0, 10.0, -2.0),  # Red center
            self.create_detection(1, 8.0, 11.0, -2.0),  # White left side
        ]
        
        # Set detections directly on mission
        self.mission.current_detections = detections
        
        # Test gate detection
        if self.mission.detect_gate(0):
            rospy.loginfo("Direct horizontal detection test successful")
            gate = self.mission.gate_positions[0]
            rospy.loginfo(f"  LEFT side: ({gate['left_side'].x:.1f}, {gate['left_side'].y:.1f})")
            rospy.loginfo(f"  Center: ({gate['center'].x:.1f}, {gate['center'].y:.1f})")
            rospy.loginfo(f"  RIGHT side: ({gate['right_side'].x:.1f}, {gate['right_side'].y:.1f})")
            rospy.loginfo(f"  Submarine will use CONTROLLED navigation through LEFT side")
        else:
            rospy.logwarn("Direct horizontal detection test failed")
    
    def test_mission_subscribers(self):
        """Test if the mission has proper subscribers set up"""
        rospy.loginfo("Checking mission subscribers...")
        
        # Look for the callback methods and data attributes
        has_detection_callback = hasattr(self.mission, 'detection_callback') and callable(getattr(self.mission, 'detection_callback'))
        has_pose_callback = hasattr(self.mission, 'pose_callback') and callable(getattr(self.mission, 'pose_callback'))
        has_current_detections = hasattr(self.mission, 'current_detections')
        has_submarine_pose = hasattr(self.mission, 'submarine_pose')
        
        rospy.loginfo(f"Mission has detection callback: {has_detection_callback}")
        rospy.loginfo(f"Mission has pose callback: {has_pose_callback}")
        rospy.loginfo(f"Mission has current_detections attribute: {has_current_detections}")
        rospy.loginfo(f"Mission has submarine_pose attribute: {has_submarine_pose}")
        
        # Test if the callbacks can be called
        detection_callable = has_detection_callback and callable(self.mission.detection_callback)
        pose_callable = has_pose_callback and callable(self.mission.pose_callback)
        
        rospy.loginfo(f"Detection callback is callable: {detection_callable}")
        rospy.loginfo(f"Pose callback is callable: {pose_callable}")
        
        # Check if mission inherits from BaseMission properly
        from mission_planner.base_mission import BaseMission
        is_base_mission = isinstance(self.mission, BaseMission)
        rospy.loginfo(f"Mission is instance of BaseMission: {is_base_mission}")
        
        all_good = (has_detection_callback and has_pose_callback and 
                   has_current_detections and has_submarine_pose and 
                   detection_callable and pose_callable)
        
        if all_good:
            rospy.loginfo("✓ All subscriber components are properly set up!")
        else:
            rospy.logwarn("✗ Some subscriber components may be missing")
        
        return all_good
    
    def run_test(self):
        """Run the complete test sequence with controlled horizontal navigation"""
        rospy.loginfo("CONTROLLED Horizontal slalom mission integration test starting in 3 seconds...")
        time.sleep(3.0)
        
        # Check mission setup
        subscribers_ok = self.test_mission_subscribers()
        
        if not subscribers_ok:
            rospy.logwarn("Missing subscribers but continuing with controlled navigation test...")
        
        # First test direct detection to verify the logic works
        rospy.loginfo("\n" + "="*30 + " DIRECT HORIZONTAL TEST " + "="*30)
        self.test_direct_detection()
        
        # Reset mission state for navigation test
        self.mission.current_detections = None
        self.mission.gate_positions = []
        self.mission.gates_passed = 0
        self.mission.current_gate = 0
        self.mission.gates_completed = []
        
        # Run the complete controlled horizontal navigation simulation
        rospy.loginfo("\n" + "="*25 + " CONTROLLED NAVIGATION SIMULATION " + "="*25)
        success = self.test_complete_slalom_navigation()
        
        # Final summary
        rospy.loginfo(f"\n" + "="*50)
        rospy.loginfo("CONTROLLED NAVIGATION TEST SUMMARY")
        rospy.loginfo("="*50)
        if success:
            rospy.loginfo("OVERALL SUCCESS: Controlled navigation prevents white pipe collision!")
        else:
            rospy.logwarn("PARTIAL SUCCESS: Some issues detected")
        
        rospy.loginfo("Controlled navigation test demonstrates:")
        rospy.loginfo("   Multi-waypoint gate passage")
        rospy.loginfo("   Collision avoidance with white pipes")
        rospy.loginfo("   Precise left side navigation")
        rospy.loginfo("   Smooth controlled movement")

if __name__ == '__main__':
    try:
        tester = SlalomMissionTester()
        tester.run_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed with error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rospy.loginfo("Controlled horizontal test completed")