#!/usr/bin/env python3

import numpy as np 
import cv2
import custom_types
from ultralytics import YOLO 
import os
import rospy 
from typing import List
from motpy import MultiObjectTracker, Detection

model = YOLO("yolo11n.pt")
tracker = MultiObjectTracker(
    dt=0.1,
    tracker_kwargs={'max_staleness': 5}
)
def process_saved_videos(video_folder: str):
    video_files = [f for f in os.listdir(video_folder) if f.endswith('.avi') or f.endswith('.mp4')] 
    for video_file in video_files:
        video_path = os.path.join(video_folder, video_file)
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            print(f"Failed to open {video_file}")
            continue

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Detect and track objects
            frame, detections = yolo_object_detection(frame)

            # Show the output frame
            cv2.imshow("YOLO + Motpy", frame)

            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        print(f"Finished processing {video_file}")

    cv2.destroyAllWindows()

def yolo_object_detection(image: np.ndarray) -> List[Detection]:
    result_list = []
    results = model(image)  # This returns a list of results

    for result in results:
        if hasattr(result, 'boxes'):  # Ensure the result has boxes
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
                conf = float(box.conf.cpu().numpy()[0])  # Convert to float
                if conf > 0.5:
                     result_list.append(Detection(box=[x1, y1, x2, y2], score=conf))

    # Motpy tracking
    tracked_objects = tracker.step(result_list)

    # Annotate frame with tracking results
    for obj in tracked_objects:
        x1, y1, x2, y2 = map(int, obj.box)
        track_id = obj.id
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, f"ID: {track_id}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return image, result_list

def calculate_point_3d(detections: List[Detection], depth_image: np.ndarray, camera_intrinsic: tuple):
    # Calculate the average depth within the bounding box
    for detection in detections:
        x_min, y_min, x_max, y_max = detection.x1, detection.y1, detection.x2, detection.y2
        if depth_image is not None:
            x_min_int = int(x_min)
            x_max_int = int(x_max)
            y_min_int = int(y_min)
            y_max_int = int(y_max)

            # Extract the depth values within the bounding box
            bbox_depth = depth_image[y_min_int:y_max_int, x_min_int:x_max_int]
            if bbox_depth.size > 0:
                # Calculate the mean depth of the bounding box
                mean_depth = np.nanmean(bbox_depth)
                if not np.isnan(mean_depth):
                    # Convert depth to 3D point using camera intrinsic parameters
                    fx, fy, cx, cy = camera_intrinsic

                    z = mean_depth
                    x_center = (x_min + x_max) / 2
                    y_center = (y_min + y_max) / 2
                    x = (x_center - cx) * z / fx
                    y = (y_center - cy) * z / fy

                    # Assign the calculated 3D point to the detection's point attribute
                    detection.point = custom_types.Point3D(x=x, y=y, z=z)
                    rospy.loginfo(f"Detection 3D point added: ({x}, {y}, {z})")
                else:
                    # Assign a default point if depth is not available
                    detection.point = custom_types.Point3D(x=0, y=0, z=0)
            else:
                detection.point = custom_types.Point3D(x=0, y=0, z=0)

def main():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    frame_folder = os.path.join(script_dir, "Videos")
    process_saved_videos(frame_folder)

if __name__ == "__main__":
    main()
    print("This is working.")