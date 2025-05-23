#!/usr/bin/env python3
"""
Monocular SLAM implementation using ORB features with 3D visualization
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import threading
import queue
import time
from typing import List, Tuple, Optional

class MonocularSLAM:
    """
    Implements a monocular SLAM system using ORB features and provides 3D visualization
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('monocular_slam', anonymous=True)
        
        # Camera parameters (adjust based on your camera)
        self.focal_length = 718.8560  # focal length in pixels
        self.pp = (607.1928, 185.2157)  # principal point (cx, cy)
        self.camera_matrix = np.array([
            [self.focal_length, 0, self.pp[0]],
            [0, self.focal_length, self.pp[1]],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
        
        # ORB detector
        self.orb = cv2.ORB_create(nfeatures=3000)
        
        # FLANN parameters for feature matching
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, 
                           table_number=6,
                           key_size=12,
                           multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        # Data for SLAM
        self.prev_frame = None
        self.prev_kp = None
        self.prev_des = None
        self.frame_count = 0
        
        # 3D visualization data
        self.camera_positions = []  # List of camera positions [x, y, z]
        self.camera_orientations = []  # List of camera orientations (rotation matrices)
        self.map_points = []  # 3D points in the map
        self.colors = []  # Colors for the 3D points
        
        # Absolute scale (for trajectory estimation)
        self.abs_scale = 1.0
        
        # Current camera pose (R|t)
        self.current_R = np.eye(3)
        self.current_t = np.zeros((3, 1))
        
        # ROS subscribers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, self.image_callback)
        
        # Publishers for visualization
        self.pose_pub = rospy.Publisher('/slam/camera_pose', PoseStamped, queue_size=10)
        
        # Thread for 3D visualization
        self.vis_queue = queue.Queue()
        self.vis_thread = threading.Thread(target=self.visualization_thread)
        self.vis_thread.daemon = True
        self.vis_thread.start()
        
        rospy.loginfo("Monocular SLAM initialized")
    
    def image_callback(self, data):
        """Process each frame from the camera"""
        try:
            # Convert ROS image to OpenCV format
            current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_frame(current_frame)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")
    
    def detect_features(self, frame):
        """Detect ORB features in a frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(gray, None)
        return gray, kp, des
    
    def match_features(self, des1, des2, kp1, kp2):
        """Match features between two frames"""
        matches = self.flann.knnMatch(des1, des2, k=2)
        
        # Apply Lowe's ratio test
        good_matches = []
        pts1 = []
        pts2 = []
        
        for match in matches:
            if len(match) == 2:  # Sometimes less than 2 matches are returned
                m, n = match
                if m.distance < 0.7 * n.distance:  # Ratio test
                    good_matches.append(m)
                    pts1.append(kp1[m.queryIdx].pt)
                    pts2.append(kp2[m.trainIdx].pt)
        
        return np.array(pts1, dtype=np.float32), np.array(pts2, dtype=np.float32), good_matches
    
    def estimate_pose(self, pts1, pts2):
        """Estimate camera pose from matched feature points"""
        if pts1.shape[0] < 5 or pts2.shape[0] < 5:
            rospy.logwarn("Not enough points for pose estimation")
            return None, None, None
        
        # Essential matrix calculation
        E, mask = cv2.findEssentialMat(pts1, pts2, self.camera_matrix, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        
        if E is None:
            rospy.logwarn("Failed to compute essential matrix")
            return None, None, None
        
        # Recover pose from essential matrix
        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, self.camera_matrix, mask=mask)
        
        return R, t, mask
    
    def triangulate_points(self, pts1, pts2, R, t):
        """Triangulate 3D points from matched features"""
        # Projection matrices
        P0 = np.hstack((np.eye(3), np.zeros((3, 1))))
        P0 = self.camera_matrix @ P0
        
        P1 = np.hstack((R, t))
        P1 = self.camera_matrix @ P1
        
        # Convert to homogeneous coordinates
        pts1_homog = cv2.convertPointsToHomogeneous(pts1)
        pts2_homog = cv2.convertPointsToHomogeneous(pts2)
        
        # Triangulate
        points_4d = cv2.triangulatePoints(P0, P1, pts1.T, pts2.T)
        
        # Convert to 3D points
        points_3d = cv2.convertPointsFromHomogeneous(points_4d.T)
        
        return points_3d
    
    def process_frame(self, frame):
        """Process a new camera frame"""
        # Detect features
        gray, kp, des = self.detect_features(frame)
        
        # Create an image with features for visualization
        feature_img = cv2.drawKeypoints(frame, kp, None, color=(0, 255, 0), 
                                       flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow('ORB Features', feature_img)
        cv2.waitKey(1)
        
        self.frame_count += 1
        
        # If this is the first frame, just save the features
        if self.prev_frame is None:
            self.prev_frame = gray
            self.prev_kp = kp
            self.prev_des = des
            
            # Initialize camera position
            self.camera_positions.append([0, 0, 0])
            self.camera_orientations.append(self.current_R.copy())
            return
        
        # Match features with previous frame
        if self.prev_des is not None and des is not None:
            pts1, pts2, matches = self.match_features(self.prev_des, des, self.prev_kp, kp)
            
            # Draw matches for visualization
            img_matches = cv2.drawMatches(self.prev_frame, self.prev_kp, gray, kp, matches[:100], None,
                                         flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            cv2.imshow('Feature Matches', img_matches)
            cv2.waitKey(1)
            
            if len(pts1) > 0 and len(pts2) > 0:
                # Estimate pose
                R, t, mask = self.estimate_pose(pts1, pts2)
                
                if R is not None and t is not None:
                    # Update camera pose
                    self.current_t = self.current_t + self.abs_scale * self.current_R @ t
                    self.current_R = R @ self.current_R
                    
                    # Save camera position and orientation
                    self.camera_positions.append(self.current_t.flatten().tolist())
                    self.camera_orientations.append(self.current_R.copy())
                    
                    # Triangulate 3D points
                    points_3d = self.triangulate_points(pts1, pts2, R, t)
                    
                    # Add valid 3D points to the map
                    for i, pt in enumerate(points_3d):
                        if mask[i]:
                            pt_global = self.current_R @ pt.T + self.current_t
                            self.map_points.append(pt_global.flatten().tolist())
                            
                            # Add color based on original image
                            x, y = int(pts1[i][0]), int(pts1[i][1])
                            if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                                color = frame[y, x].tolist()
                                self.colors.append(color)
                            else:
                                self.colors.append([0, 255, 0])  # Default to green
                    
                    # Publish camera pose for external visualization
                    self.publish_pose()
                    
                    # Add data for visualization thread
                    if self.frame_count % 5 == 0:  # Update visualization every 5 frames
                        self.vis_queue.put((
                            self.camera_positions.copy(),
                            self.map_points.copy(),
                            self.colors.copy()
                        ))
        
        # Update previous frame data
        self.prev_frame = gray
        self.prev_kp = kp
        self.prev_des = des
    
    def publish_pose(self):
        """Publish current camera pose to ROS"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        
        # Convert rotation matrix to quaternion
        import tf.transformations
        quaternion = tf.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((self.current_R, self.current_t)), [0, 0, 0, 1]))
        )
        
        # Set position
        pose_msg.pose.position.x = self.current_t[0, 0]
        pose_msg.pose.position.y = self.current_t[1, 0]
        pose_msg.pose.position.z = self.current_t[2, 0]
        
        # Set orientation
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        self.pose_pub.publish(pose_msg)
    
    def visualization_thread(self):
        """Thread for 3D visualization of camera trajectory and map points"""
        plt.ion()
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Initial setup
        camera_line, = ax.plot([], [], [], 'b-', linewidth=2, label='Camera Path')
        points_scatter = ax.scatter([], [], [], s=1, c=[], marker='o', label='Map Points')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Monocular SLAM Visualization')
        ax.legend()
        
        while not rospy.is_shutdown():
            try:
                # Get new data with timeout
                data = self.vis_queue.get(timeout=0.5)
                
                camera_positions, map_points, colors = data
                
                if not camera_positions:
                    continue
                
                # Extract coordinates for plotting
                cam_x = [pos[0] for pos in camera_positions]
                cam_y = [pos[1] for pos in camera_positions]
                cam_z = [pos[2] for pos in camera_positions]
                
                # Clear previous data and update plot
                ax.clear()
                
                # Plot camera trajectory
                ax.plot(cam_x, cam_y, cam_z, 'b-', linewidth=2, label='Camera Path')
                
                # Plot current camera position
                ax.scatter(cam_x[-1], cam_y[-1], cam_z[-1], color='red', s=100, marker='x', label='Current Position')
                
                # Plot map points (if we have some)
                if map_points:
                    point_x = [pt[0] for pt in map_points]
                    point_y = [pt[1] for pt in map_points]
                    point_z = [pt[2] for pt in map_points]
                    
                    # Convert color list to format required by scatter
                    rgb_colors = np.array(colors) / 255.0
                    
                    ax.scatter(point_x, point_y, point_z, s=1, c=rgb_colors, marker='o', label='Map Points')
                
                # Set axes limits dynamically based on data
                if cam_x and cam_y and cam_z:
                    max_range = np.max([
                        np.max(np.abs(cam_x)),
                        np.max(np.abs(cam_y)),
                        np.max(np.abs(cam_z))
                    ])
                    
                    ax.set_xlim([-max_range, max_range])
                    ax.set_ylim([-max_range, max_range])
                    ax.set_zlim([-max_range, max_range])
                
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title(f'Monocular SLAM: {len(map_points)} points, Frame #{self.frame_count}')
                ax.legend()
                
                # Draw the updated plot
                fig.canvas.draw_idle()
                plt.pause(0.01)
                
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Visualization error: {e}")
                continue
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Starting Monocular SLAM...")
        
        # If you want to run from a video file instead of ROS
        # self.run_from_video("/path/to/video.mp4")
        
        rospy.spin()
    
    def run_from_video(self, video_path):
        """Run SLAM on a video file"""
        cap = cv2.VideoCapture(video_path)
        
        if not cap.isOpened():
            rospy.logerr(f"Error: Could not open video file {video_path}")
            return
        
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            
            if not ret:
                rospy.loginfo("End of video file")
                break
            
            self.process_frame(frame)
            time.sleep(0.03)  # Simulate 30 fps
        
        cap.release()

def main():
    """Main function to start the SLAM system"""
    try:
        slam = MonocularSLAM()
        slam.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()