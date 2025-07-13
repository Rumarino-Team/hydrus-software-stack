#!/usr/bin/env python3
"""
ROS Profiler Demo

Complete demonstration of how to use the ROS profiler with instrumented nodes.
This script creates example ROS nodes and shows how to monitor them with the profiler.

Usage:
    # Terminal 1: Start the demo nodes
    python3 ros_profiler_demo.py

    # Terminal 2: Run the profiler to monitor the demo nodes
    python3 ros_profiler.py --nodes image_processor data_analyzer --export demo_profile.csv

    # Or monitor all nodes
    python3 ros_profiler.py --export demo_profile.csv
"""

import atexit
import math
import os
import random
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Header, String

# Add the services directory to the path for imports
sys.path.append(os.path.dirname(__file__))
from profiling_decorators import (
    FunctionProfiler,
    ProfileManager,
    export_profiling_data,
    profile_function,
    profile_method,
    profile_ros_callback,
)


class DemoImageProcessor:
    """
    Demo ROS node that simulates image processing with profiling decorators.
    This represents a typical computer vision node in the Hydrus stack.
    """

    def __init__(self):
        # Initialize profiling for this node
        ProfileManager.get_instance().set_node_name("image_processor")
        
        rospy.loginfo("Starting Image Processor Demo Node")

        # Publishers
        self.processed_image_pub = rospy.Publisher(
            "/processed_image", Image, queue_size=1
        )
        self.detection_count_pub = rospy.Publisher(
            "/detection_count", Float32, queue_size=1
        )
        self.status_pub = rospy.Publisher("/processor_status", String, queue_size=1)

        # Subscribers
        self.raw_image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.image_callback, queue_size=1
        )

        # Internal state
        self.frame_count = 0
        self.processing_enabled = True
        self.last_detection_time = time.time()

        # Start background processing thread
        self.processing_thread = threading.Thread(target=self._background_processing)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        rospy.loginfo("Image Processor initialized successfully")

    @profile_ros_callback("/camera/image_raw")
    def image_callback(self, msg):
        """Main image processing callback with profiling"""
        self.frame_count += 1
        
        if not self.processing_enabled:
            return

        try:
            # Simulate image conversion
            image_data = self._convert_ros_image(msg)
            
            # Simulate object detection
            detections = self._detect_objects(image_data)
            
            # Simulate image enhancement
            enhanced_image = self._enhance_image(image_data)
            
            # Publish results
            self._publish_results(enhanced_image, len(detections))
            
        except Exception as e:
            rospy.logerr(f"Error in image processing: {e}")

    @profile_method("image_conversion", "cv")
    def _convert_ros_image(self, msg):
        """Simulate ROS image to OpenCV conversion"""
        # Simulate processing time
        time.sleep(random.uniform(0.001, 0.005))
        
        # Create mock image data
        height, width = msg.height, msg.width
        if height == 0 or width == 0:
            height, width = 480, 640
            
        image_data = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
        return image_data

    @profile_function("object_detection", "cv")
    def _detect_objects(self, image_data):
        """Simulate object detection processing"""
        with FunctionProfiler("cv.yolo_inference"):
            # Simulate YOLO inference time
            time.sleep(random.uniform(0.02, 0.08))
            
        with FunctionProfiler("cv.post_processing"):
            # Simulate post-processing
            time.sleep(random.uniform(0.005, 0.015))
            
        # Generate random detections
        num_detections = random.randint(0, 5)
        detections = []
        
        for i in range(num_detections):
            detection = {
                "class": random.choice(["person", "car", "bicycle", "dog"]),
                "confidence": random.uniform(0.5, 0.95),
                "bbox": [
                    random.randint(0, 100),
                    random.randint(0, 100),
                    random.randint(50, 200),
                    random.randint(50, 200),
                ],
            }
            detections.append(detection)
            
        return detections

    @profile_method("image_enhancement", "cv")
    def _enhance_image(self, image_data):
        """Simulate image enhancement processing"""
        # Simulate filtering operations
        time.sleep(random.uniform(0.01, 0.03))
        
        # Mock enhancement
        enhanced = image_data.copy()
        return enhanced

    @profile_method("publishing", "ros")
    def _publish_results(self, image_data, detection_count):
        """Publish processing results"""
        # Create and publish processed image message
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "camera"
        img_msg.height = image_data.shape[0]
        img_msg.width = image_data.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.data = image_data.tobytes()
        
        self.processed_image_pub.publish(img_msg)
        
        # Publish detection count
        count_msg = Float32()
        count_msg.data = float(detection_count)
        self.detection_count_pub.publish(count_msg)

    def _background_processing(self):
        """Background thread for periodic tasks"""
        rate = rospy.Rate(2)  # 2 Hz
        
        while not rospy.is_shutdown():
            try:
                with FunctionProfiler("background.status_update"):
                    # Publish status
                    status_msg = String()
                    status_msg.data = f"Processing frame {self.frame_count}, enabled: {self.processing_enabled}"
                    self.status_pub.publish(status_msg)
                
                with FunctionProfiler("background.maintenance"):
                    # Simulate periodic maintenance tasks
                    time.sleep(random.uniform(0.01, 0.02))
                    
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in background processing: {e}")


class DemoDataAnalyzer:
    """
    Demo ROS node that simulates data analysis with profiling decorators.
    This represents a typical data processing node in the Hydrus stack.
    """

    def __init__(self):
        # Initialize profiling for this node
        ProfileManager.get_instance().set_node_name("data_analyzer")
        
        rospy.loginfo("Starting Data Analyzer Demo Node")

        # Publishers
        self.analysis_pub = rospy.Publisher("/analysis_result", String, queue_size=10)
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Subscribers
        self.detection_sub = rospy.Subscriber(
            "/detection_count", Float32, self.detection_callback, queue_size=10
        )
        self.status_sub = rospy.Subscriber(
            "/processor_status", String, self.status_callback, queue_size=10
        )

        # Internal state
        self.detection_history = []
        self.analysis_count = 0

        # Start analysis timer
        self.analysis_timer = rospy.Timer(
            rospy.Duration(0.5), self._periodic_analysis  # 2 Hz
        )

        rospy.loginfo("Data Analyzer initialized successfully")

    @profile_ros_callback("/detection_count")
    def detection_callback(self, msg):
        """Process detection count updates"""
        with FunctionProfiler("data.detection_processing"):
            self.detection_history.append(msg.data)
            
            # Keep only recent history
            if len(self.detection_history) > 100:
                self.detection_history.pop(0)
                
            # Simulate processing
            time.sleep(random.uniform(0.001, 0.003))

    @profile_ros_callback("/processor_status")
    def status_callback(self, msg):
        """Process status updates"""
        with FunctionProfiler("data.status_processing"):
            # Simulate status analysis
            time.sleep(random.uniform(0.0005, 0.002))

    def _periodic_analysis(self, event):
        """Periodic analysis function called by timer"""
        try:
            self.analysis_count += 1
            
            # Perform statistical analysis
            stats = self._calculate_statistics()
            
            # Generate control commands
            velocity = self._generate_velocity_command(stats)
            
            # Publish results
            self._publish_analysis(stats, velocity)
            
        except Exception as e:
            rospy.logerr(f"Error in periodic analysis: {e}")

    @profile_function("statistical_analysis", "analysis")
    def _calculate_statistics(self):
        """Calculate statistics from detection history"""
        if not self.detection_history:
            return {"avg": 0, "max": 0, "min": 0, "trend": "stable"}
            
        # Simulate computation time
        time.sleep(random.uniform(0.005, 0.015))
        
        avg_detections = sum(self.detection_history) / len(self.detection_history)
        max_detections = max(self.detection_history)
        min_detections = min(self.detection_history)
        
        # Simple trend analysis
        if len(self.detection_history) >= 10:
            recent_avg = sum(self.detection_history[-5:]) / 5
            older_avg = sum(self.detection_history[-10:-5]) / 5
            
            if recent_avg > older_avg * 1.2:
                trend = "increasing"
            elif recent_avg < older_avg * 0.8:
                trend = "decreasing"
            else:
                trend = "stable"
        else:
            trend = "insufficient_data"
            
        return {
            "avg": avg_detections,
            "max": max_detections,
            "min": min_detections,
            "trend": trend,
        }

    @profile_function("velocity_generation", "control")
    def _generate_velocity_command(self, stats):
        """Generate velocity commands based on analysis"""
        # Simulate control algorithm computation
        time.sleep(random.uniform(0.002, 0.008))
        
        velocity = Twist()
        
        # Simple reactive behavior based on detection statistics
        if stats["avg"] > 3:
            # Many detections - slow down
            velocity.linear.x = 0.1
            velocity.angular.z = 0.0
        elif stats["avg"] < 1:
            # Few detections - speed up
            velocity.linear.x = 0.5
            velocity.angular.z = 0.0
        else:
            # Normal operation
            velocity.linear.x = 0.3
            velocity.angular.z = random.uniform(-0.1, 0.1)
            
        return velocity

    @profile_method("result_publishing", "ros")
    def _publish_analysis(self, stats, velocity):
        """Publish analysis results and velocity commands"""
        # Publish analysis result
        analysis_msg = String()
        analysis_msg.data = f"Analysis #{self.analysis_count}: avg={stats['avg']:.1f}, trend={stats['trend']}"
        self.analysis_pub.publish(analysis_msg)
        
        # Publish velocity command
        self.velocity_pub.publish(velocity)


class DemoImagePublisher:
    """
    Demo ROS node that publishes synthetic images to simulate a camera.
    """

    def __init__(self):
        rospy.loginfo("Starting Demo Camera Node")

        # Publisher
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)

        # Start publishing timer
        self.pub_timer = rospy.Timer(
            rospy.Duration(1.0 / 30.0), self._publish_image  # 30 FPS
        )

        rospy.loginfo("Demo Camera initialized successfully")

    def _publish_image(self, event):
        """Publish synthetic image data"""
        try:
            # Create synthetic image
            height, width = 480, 640
            image_data = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
            
            # Add some patterns to make it more realistic
            cv2.circle(image_data, (width//2, height//2), 50, (255, 0, 0), -1)
            cv2.rectangle(image_data, (100, 100), (200, 200), (0, 255, 0), 2)
            
            # Create ROS message
            img_msg = Image()
            img_msg.header = Header()
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera"
            img_msg.height = height
            img_msg.width = width
            img_msg.encoding = "bgr8"
            img_msg.step = width * 3
            img_msg.data = image_data.tobytes()
            
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")


def main():
    """Main function to run the demo"""
    import argparse
    
    parser = argparse.ArgumentParser(description="ROS Profiler Demo Nodes")
    parser.add_argument("node_type", nargs="?", default="all", 
                       choices=["all", "camera", "processor", "analyzer"],
                       help="Type of node to run (default: all)")
    
    # Parse known args only to ignore ROS arguments
    args, unknown = parser.parse_known_args()
    
    # Filter out ROS-specific arguments from sys.argv so rospy doesn't see them
    import sys
    filtered_argv = [sys.argv[0]]  # Keep script name
    if hasattr(args, 'node_type') and args.node_type != "all":
        filtered_argv.append(args.node_type)
    sys.argv = filtered_argv
    
    try:
        if args.node_type == "camera":
            rospy.init_node("demo_camera", anonymous=True)
            rospy.loginfo("Starting ROS Profiler Demo - Camera Only")
            
            # Set up profiling data export for camera
            def export_camera_data():
                try:
                    export_profiling_data("demo_camera_profile.json")
                    rospy.loginfo("Camera profiling data exported")
                except Exception as e:
                    rospy.logerr(f"Error exporting camera data: {e}")
            atexit.register(export_camera_data)
            
            camera = DemoImagePublisher()
            rospy.loginfo("Demo camera started successfully!")
            rospy.loginfo("Monitor with: python3 ros_profiler.py --nodes demo_camera --export camera_profile.csv")
            rospy.spin()
            
        elif args.node_type == "processor":
            rospy.init_node("image_processor", anonymous=True)
            rospy.loginfo("Starting ROS Profiler Demo - Processor Only")
            
            # Set up profiling data export for processor
            def export_processor_data():
                try:
                    export_profiling_data("demo_processor_profile.json")
                    rospy.loginfo("Processor profiling data exported")
                except Exception as e:
                    rospy.logerr(f"Error exporting processor data: {e}")
            atexit.register(export_processor_data)
            
            processor = DemoImageProcessor()
            rospy.loginfo("Demo processor started successfully!")
            rospy.loginfo("Monitor with: python3 ros_profiler.py --nodes image_processor --export processor_profile.csv")
            rospy.spin()
            
        elif args.node_type == "analyzer":
            rospy.init_node("data_analyzer", anonymous=True)
            rospy.loginfo("Starting ROS Profiler Demo - Analyzer Only")
            
            # Set up profiling data export for analyzer
            def export_analyzer_data():
                try:
                    export_profiling_data("demo_analyzer_profile.json")
                    rospy.loginfo("Analyzer profiling data exported")
                except Exception as e:
                    rospy.logerr(f"Error exporting analyzer data: {e}")
            atexit.register(export_analyzer_data)
            
            analyzer = DemoDataAnalyzer()
            rospy.loginfo("Demo analyzer started successfully!")
            rospy.loginfo("Monitor with: python3 ros_profiler.py --nodes data_analyzer --export analyzer_profile.csv")
            rospy.spin()
            
        else:  # args.node_type == "all"
            rospy.loginfo("ERROR: Cannot run all nodes in the same process!")
            rospy.loginfo("Please run each node in a separate terminal:")
            rospy.loginfo("  Terminal 1: python3 ros_profiler_demo.py camera")
            rospy.loginfo("  Terminal 2: python3 ros_profiler_demo.py processor") 
            rospy.loginfo("  Terminal 3: python3 ros_profiler_demo.py analyzer")
            rospy.loginfo("Or use the launch file: roslaunch autonomy profiler_demo.launch")
            return
        
    except KeyboardInterrupt:
        rospy.loginfo("Demo stopped by user")
    except Exception as e:
        rospy.logerr(f"Error in demo: {e}")


if __name__ == "__main__":
    main()
