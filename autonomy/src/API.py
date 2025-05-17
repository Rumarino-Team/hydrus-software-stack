import uvicorn
import asyncio
import rospy
import cv2
import numpy as np
import json
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from sensor_msgs.msg import Image
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Dict, Any
from autonomy.msg import Detections

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:5000"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global variables
rgb_image = None
latest_detections = []  # Store the latest detections from cv_publishers

def rgb_image_callback(msg):
    global rgb_image
    try:
        height = msg.height
        width = msg.width

        img_array = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.encoding in ["rgb8", "bgr8"]:
            channels = 3
        elif msg.encoding in ["rgba8", "bgra8"]:
            channels = 4
        elif msg.encoding == "mono8":
            channels = 1
        else:
            rospy.logwarn(f"Unsupported encoding: {msg.encoding}")
            return

        expected_size = height * width * channels
        if img_array.size != expected_size:
            rospy.logwarn(f"Unexpected image size: got {img_array.size}, expected {expected_size}")
            return

        img_array = img_array.reshape((height, width, channels))

        # Remove alpha channel if present
        if channels == 4:
            img_array = img_array[:, :, :3]

        rgb_image = img_array

    except Exception as e:
        rospy.logerr(f"Error converting image: {e}")

def detection_callback(msg):
    """Callback for the /detector/box_detection topic"""
    global latest_detections
    
    # Check if this detection is from a new detector not already in our list
    detector_name = msg.detector_name
    
    # Find if we already have detections from this detector
    found = False
    for i, detection_data in enumerate(latest_detections):
        if detection_data.get('detector_name') == detector_name:
            # Update existing detector entry
            latest_detections[i] = convert_detection_msg_to_dict(msg)
            found = True
            break
            
    # If not found, add it to the list
    if not found:
        latest_detections.append(convert_detection_msg_to_dict(msg))
    
    rospy.logdebug(f"Received detections from {detector_name}: {len(msg.detections)} objects")

def convert_detection_msg_to_dict(msg):
    """Convert a ROS Detections message to a dictionary for JSON serialization"""
    converted_detections = []
    
    for det_msg in msg.detections:
        converted_detections.append({
            "cls": det_msg.cls,
            "confidence": det_msg.confidence,
            "point": {
                "x": det_msg.point.x,
                "y": det_msg.point.y,
                "z": det_msg.point.z
            },
            "bounding_box": {
                "x_offset": det_msg.bounding_box.x_offset,
                "y_offset": det_msg.bounding_box.y_offset,
                "width":   det_msg.bounding_box.width,
                "height":  det_msg.bounding_box.height
            }
        })
        
    return {
        "detector_name": msg.detector_name,
        "detections": converted_detections
    }

def generate_video_stream():
    """Generate frames for video streaming"""
    while True:
        if rgb_image is not None:
            _, jpeg = cv2.imencode('.jpg', rgb_image)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        else:
            rospy.logwarn("No image received yet")
            # Wait a bit before trying again
            asyncio.sleep(0.1)

@app.get("/video_feed")
def video_feed():
    """Endpoint to stream video frames"""
    return StreamingResponse(generate_video_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

async def sse_generator():
    """Generate Server-Sent Events with detection data"""
    while True:
        # Use the latest detections received from callbacks
        json_str = json.dumps(latest_detections)
        yield f"data: {json_str}\n\n"
        await asyncio.sleep(0.1)

@app.get("/detections/stream")
def get_detections_stream():
    """Endpoint to stream detection data as server-sent events"""
    return StreamingResponse(sse_generator(), media_type="text/event-stream")

if __name__ == "__main__":
    rospy.init_node('detection_api', anonymous=True)
    
    # Subscribe to the video feed
    rgb_topic = rospy.get_param('~rgb_image_topic', '/zed2i/zed_node/rgb/image_rect_color')
    rospy.Subscriber(rgb_topic, Image, rgb_image_callback)
    
    # Subscribe to the detections from cv_publishers.py
    rospy.Subscriber('/detector/box_detection', Detections, detection_callback)
    
    rospy.loginfo(f"API server initialized. Subscribing to RGB image: {rgb_topic}")
    rospy.loginfo("Subscribing to detections from cv_publishers at: /detector/box_detection")
    
    # Start the FastAPI server
    uvicorn.run(app, host="0.0.0.0", port=8000)