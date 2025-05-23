#!/usr/bin/env python3
"""
FastAPI + ROS1 bidirectional WebSocket bridge for Godot.
Handles simulation data exchange between ROS and Godot using WebSockets.

CONNECTION OVERVIEW:
------------------

1. GODOT → ROS DATA FLOW:
   * Godot sends data via WebSocket (/ws endpoint)
   * Data types received:
     - "state" - Submarine position/orientation → Published as /sim/odom
     - "camera" - RGB camera images → Published as /sim/camera/rgb/image_raw
     - "depth" - Depth camera data → Published as /sim/camera/depth/image_raw
     - "log" - Logging messages → Printed via rospy.loginfo
   * Functions involved:
     - websocket_endpoint() - Handles incoming WebSocket connections and messages
     - godot_state_to_odom() - Converts Godot state data to ROS Odometry messages
     - godot_camera_to_image() - Converts Godot camera data to ROS Image messages
     - godot_depth_to_image() - Converts Godot depth data to ROS Image messages

2. ROS → GODOT DATA FLOW:
   * ROS messages are converted and sent to Godot via WebSocket
   * Data types sent:
     - "cmd_vel" - Velocity commands from /cmd_vel topic
     - "thruster" - Individual thruster commands from /hydrus/thrusters/[1-8] topics
     - "depth_control" - Depth control from /hydrus/depth topic
     - "torpedo_control" - Torpedo control from /hydrus/torpedo topic
   * Functions involved:
     - cmd_vel_callback() - Processes incoming ROS Twist messages
     - thruster_callback() - Processes individual thruster commands
     - depth_control_callback() - Processes depth control commands
     - torpedo_control_callback() - Processes torpedo commands
     - ws_producer() - Manages sending data to Godot clients
     - process_cmd_queue() - Sends command velocity data to Godot
     - process_thruster_queue() - Sends thruster control data to Godot

3. WEB VISUALIZATION:
   * The FastAPI app provides endpoints for web visualization:
     - /video_feed - Streams camera feed as MJPEG (HTTP)
     - /detections/stream - Streams detection data as Server-Sent Events (SSE)
     - /simulation_status - Returns if simulation is active
   * Functions involved:
     - video_feed() - Endpoint that streams camera images as MJPEG
     - generate_video_stream() - Generates video frames for streaming
     - detections_stream() - Endpoint that streams detection data as SSE
     - sse_generator() - Generates SSE events with detection data
     - simulation_status() - Endpoint to check simulation status

4. ROS MESSAGE PROCESSING:
   * ROS messages are processed for web visualization and Godot communications:
     - rgb_image_callback() - Processes RGB images from ROS
     - depth_image_callback() - Processes depth images from ROS
     - detection_callback() - Processes detection messages from ROS
     - convert_detection_msg_to_dict() - Converts ROS Detection messages to JSON-serializable dict
"""

import asyncio
import json
import time
import cv2
import numpy as np
import rospy
import uvicorn
from typing import List, Dict, Any, Set, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int16
from autonomy.msg import Detections

# ─────────────────────────────  FASTAPI APP  ──────────────────────────────
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ─────────────────────────────  GLOBAL STATE  ─────────────────────────────
rgb_image: Optional[np.ndarray] = None
depth_image: Optional[np.ndarray] = None
latest_detections: List[Dict[str, Any]] = []
simulation_active: bool = False

# WebSocket bookkeeping
clients: Set[WebSocket] = set()
cmd_vel_queue: "asyncio.Queue[Dict]" = asyncio.Queue(maxsize=10)
thruster_queue: "asyncio.Queue[Dict]" = asyncio.Queue(maxsize=10)

# ─────────────────────────  ROS CALLBACKS & HELPERS  ─────────────────────
def rgb_image_callback(msg: Image) -> None:
    """Process incoming RGB images from ROS"""
    global rgb_image
    try:
        h, w = msg.height, msg.width
        enc = msg.encoding

        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
        if channels is None:
            rospy.logwarn(f"Unsupported encoding: {enc}")
            return
            
        expected = h * w * channels
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size != expected:
            rospy.logwarn(f"Image data size mismatch ({buf.size} vs {expected})")
            return
            
        img = buf.reshape((h, w, channels))
        if channels == 4:  # strip alpha
            img = img[:, :, :3]
        rgb_image = img
    except Exception as e:
        rospy.logerr(f"Error processing RGB image: {e}")


def depth_image_callback(msg: Image) -> None:
    """Process incoming depth images from ROS"""
    global depth_image
    try:
        h, w = msg.height, msg.width
        enc = msg.encoding
        
        if enc != "32FC1":  # Standard encoding for depth images
            rospy.logwarn(f"Unexpected depth encoding: {enc}")
            
        # Convert Float32 depth data
        buf = np.frombuffer(msg.data, dtype=np.float32)
        expected = h * w
        if buf.size != expected:
            rospy.logwarn(f"Depth data size mismatch ({buf.size} vs {expected})")
            return
            
        depth_image = buf.reshape((h, w))
    except Exception as e:
        rospy.logerr(f"Error processing depth image: {e}")


def detection_callback(msg: Detections) -> None:
    """Update latest_detections list in-place."""
    global latest_detections
    as_dict = convert_detection_msg_to_dict(msg)
    for i, d in enumerate(latest_detections):
        if d["detector_name"] == msg.detector_name:
            latest_detections[i] = as_dict
            break
    else:
        latest_detections.append(as_dict)


def convert_detection_msg_to_dict(msg: Detections) -> Dict[str, Any]:
    """Convert ROS Detection messages to Python dict for JSON serialization"""
    return {
        "detector_name": msg.detector_name,
        "detections": [
            {
                "cls": d.cls,
                "confidence": d.confidence,
                "point": {"x": d.point.x, "y": d.point.y, "z": d.point.z},
                "bounding_box": {
                    "x_offset": d.bounding_box.x_offset,
                    "y_offset": d.bounding_box.y_offset,
                    "width": d.bounding_box.width,
                    "height": d.bounding_box.height,
                },
            }
            for d in msg.detections
        ],
    }


def cmd_vel_callback(msg: Twist) -> None:
    """Process incoming cmd_vel messages from ROS"""
    try:
        # Convert Twist message to dict for WebSocket transmission
        cmd = {
            "linear": {
                "x": float(msg.linear.x),
                "y": float(msg.linear.y),
                "z": float(msg.linear.z)
            },
            "angular": {
                "x": float(msg.angular.x),
                "y": float(msg.angular.y),
                "z": float(msg.angular.z)
            }
        }
        cmd_vel_queue.put_nowait({"cmd_vel": cmd})
    except asyncio.QueueFull:
        rospy.logwarn("cmd_vel queue full, dropping message")
    except Exception as e:
        rospy.logerr(f"Error processing cmd_vel: {e}")


def thruster_callback(msg: Int16, index: int) -> None:
    """Process individual thruster PWM values"""
    try:
        thruster_queue.put_nowait({"thruster": {"index": index, "value": msg.data}})
    except asyncio.QueueFull:
        pass  # Silently drop if queue is full


def depth_control_callback(msg: Int16) -> None:
    """Process depth control PWM values"""
    try:
        thruster_queue.put_nowait({"depth_control": msg.data})
    except asyncio.QueueFull:
        pass


def torpedo_control_callback(msg: Int16) -> None:
    """Process torpedo control PWM values"""
    try:
        thruster_queue.put_nowait({"torpedo_control": msg.data})
    except asyncio.QueueFull:
        pass


# ─────────────────────────  GODOT → ROS CONVERSIONS  ─────────────────────
def godot_state_to_odom(state: Dict[str, Any]) -> Odometry:
    """Convert JSON dict from Godot to nav_msgs/Odometry."""
    odom = Odometry()
    odom.header = Header(stamp=rospy.Time.now(), frame_id="odom")
    odom.child_frame_id = "base_link"

    pos = state.get("pos", [0, 0, 0])
    odom.pose.pose.position.x = pos[0]
    odom.pose.pose.position.y = pos[1]
    odom.pose.pose.position.z = pos[2]

    qt = state.get("orient", [0, 0, 0, 1])
    odom.pose.pose.orientation.x = qt[0]
    odom.pose.pose.orientation.y = qt[1]
    odom.pose.pose.orientation.z = qt[2]
    odom.pose.pose.orientation.w = qt[3]

    vel = state.get("vel", [0, 0, 0])
    odom.twist.twist.linear.x = vel[0]
    odom.twist.twist.linear.y = vel[1]
    odom.twist.twist.linear.z = vel[2]
    
    ang_vel = state.get("ang_vel", [0, 0, 0])
    odom.twist.twist.angular.x = ang_vel[0]
    odom.twist.twist.angular.y = ang_vel[1]
    odom.twist.twist.angular.z = ang_vel[2]
    
    return odom


def godot_camera_to_image(camera_data: Dict[str, Any]) -> Image:
    """Convert camera data from Godot to ROS Image message."""
    if "data" not in camera_data or "width" not in camera_data or "height" not in camera_data:
        raise ValueError("Camera data missing required fields")
    
    # Decode base64 image data from Godot
    import base64
    raw_data = base64.b64decode(camera_data["data"])
    
    # Create Image message
    img_msg = Image()
    img_msg.header = Header(stamp=rospy.Time.now(), frame_id="camera_sim")
    img_msg.height = camera_data["height"]
    img_msg.width = camera_data["width"]
    img_msg.encoding = camera_data.get("encoding", "bgr8")
    img_msg.is_bigendian = False
    img_msg.step = img_msg.width * (3 if img_msg.encoding in ["rgb8", "bgr8"] else 1)
    img_msg.data = raw_data
    
    return img_msg


def godot_depth_to_image(depth_data: Dict[str, Any]) -> Image:
    """Convert depth data from Godot to ROS depth Image message."""
    if "data" not in depth_data or "width" not in depth_data or "height" not in depth_data:
        raise ValueError("Depth data missing required fields")
    
    # Decode base64 depth data from Godot
    import base64
    raw_data = base64.b64decode(depth_data["data"])
    
    # Create Image message for depth
    img_msg = Image()
    img_msg.header = Header(stamp=rospy.Time.now(), frame_id="camera_depth_sim")
    img_msg.height = depth_data["height"]
    img_msg.width = depth_data["width"]
    img_msg.encoding = "32FC1"  # Standard for depth images
    img_msg.is_bigendian = False
    img_msg.step = img_msg.width * 4  # 4 bytes per float32
    img_msg.data = raw_data
    
    return img_msg


# ─────────────────────────  FASTAPI STREAM ENDPOINTS  ─────────────────────
def generate_video_stream():
    """Generate frames for video streaming."""
    while True:
        if rgb_image is not None:
            try:
                ok, jpeg = cv2.imencode(".jpg", rgb_image)
                if ok:
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + jpeg.tobytes()
                        + b"\r\n"
                    )
            except Exception as e:
                rospy.logerr(f"Error encoding video frame: {e}")
        time.sleep(0.033)  # ~30fps


@app.get("/video_feed")
def video_feed():
    """Endpoint to serve camera feed as MJPEG stream."""
    return StreamingResponse(
        generate_video_stream(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


async def sse_generator():
    """Generate server-sent events with detection data."""
    while True:
        yield f"data: {json.dumps(latest_detections)}\n\n"
        await asyncio.sleep(0.1)


@app.get("/detections/stream")
def detections_stream():
    """Endpoint to stream detection data as SSE."""
    return StreamingResponse(sse_generator(), media_type="text/event-stream")


@app.get("/simulation_status")
def simulation_status():
    """Endpoint to check if simulation is active."""
    return {"active": simulation_active}


# ────────────────────────  WS ENDPOINT & HANDLERS  ────────────────────────
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """WebSocket endpoint for bidirectional communication with Godot."""
    global simulation_active
    await ws.accept()
    clients.add(ws)
    simulation_active = True
    rospy.loginfo("Godot WebSocket connected")
    
    try:
        # Launch background producer coroutine for this client
        producer_task = asyncio.create_task(ws_producer(ws))
        
        # Process incoming messages from Godot
        async for text in ws.iter_text():
            try:
                data = json.loads(text)
                
                # Handle different message types
                if "state" in data:
                    state = data["state"]
                    odom_pub.publish(godot_state_to_odom(state))
                    
                elif "camera" in data:
                    try:
                        camera_data = data["camera"]
                        img_msg = godot_camera_to_image(camera_data)
                        camera_pub.publish(img_msg)
                    except Exception as e:
                        rospy.logwarn(f"Camera data processing error: {e}")
                        
                elif "depth" in data:
                    try:
                        depth_data = data["depth"]
                        depth_msg = godot_depth_to_image(depth_data)
                        depth_pub.publish(depth_msg)
                    except Exception as e:
                        rospy.logwarn(f"Depth data processing error: {e}")
                        
                elif "log" in data:
                    rospy.loginfo(f"Godot: {data['log']}")
                    
            except json.JSONDecodeError:
                rospy.logwarn("Invalid JSON received from Godot")
            except Exception as e:
                rospy.logwarn(f"Error processing Godot message: {e}")
                
    except WebSocketDisconnect:
        rospy.loginfo("Godot WebSocket disconnected")
    except Exception as e:
        rospy.logerr(f"WebSocket error: {e}")
    finally:
        simulation_active = len(clients) > 1  # Still active if other clients connected
        producer_task.cancel()
        clients.discard(ws)


async def ws_producer(ws: WebSocket):
    """Send ROS messages to the WebSocket client."""
    try:
        cmd_task = asyncio.create_task(process_cmd_queue(ws))
        thruster_task = asyncio.create_task(process_thruster_queue(ws))
        
        # Wait for both tasks to complete (which should be never)
        await asyncio.gather(cmd_task, thruster_task)
    except asyncio.CancelledError:
        # Clean cancellation when the client disconnects
        pass
    except Exception as e:
        rospy.logerr(f"Producer error: {e}")


async def process_cmd_queue(ws: WebSocket):
    """Process and send cmd_vel messages to Godot."""
    while True:
        cmd = await cmd_vel_queue.get()
        try:
            await ws.send_text(json.dumps(cmd))
        except Exception as e:
            rospy.logwarn(f"Error sending cmd_vel to Godot: {e}")
            break


async def process_thruster_queue(ws: WebSocket):
    """Process and send thruster control messages to Godot."""
    while True:
        thruster_data = await thruster_queue.get()
        try:
            await ws.send_text(json.dumps(thruster_data))
        except Exception as e:
            rospy.logwarn(f"Error sending thruster data to Godot: {e}")
            break


# ─────────────────────────────────  MAIN  ─────────────────────────────────
def main():
    """Initialize ROS node and start FastAPI server."""
    global odom_pub, camera_pub, depth_pub
    
    rospy.init_node("godot_bridge", anonymous=True)
    rospy.loginfo("Starting Godot-ROS bridge...")
    
    # Publishers for simulator data
    odom_pub = rospy.Publisher("/sim/odom", Odometry, queue_size=10)
    camera_pub = rospy.Publisher("/sim/camera/rgb/image_raw", Image, queue_size=5)
    depth_pub = rospy.Publisher("/sim/camera/depth/image_raw", Image, queue_size=5)
    
    # Subscribe to control topics
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/submarine/cmd_vel", Twist, cmd_vel_callback)
    
    # Subscribe to thruster topics
    for i in range(8):
        rospy.Subscriber(f"/hydrus/thrusters/{i+1}", Int16, 
                        lambda msg, idx=i+1: thruster_callback(msg, idx))
    
    # Subscribe to depth and torpedo control
    rospy.Subscriber("/hydrus/depth", Int16, depth_control_callback)
    rospy.Subscriber("/hydrus/torpedo", Int16, torpedo_control_callback)
    
    # Subscribe to actual camera topics to stream them in web interface
    rgb_topic = rospy.get_param("~rgb_image_topic", "/zed2i/zed_node/rgb/image_rect_color")
    depth_topic = rospy.get_param("~depth_image_topic", "/zed2i/zed_node/depth/depth_registered")
    
    rospy.Subscriber(rgb_topic, Image, rgb_image_callback)
    rospy.Subscriber(depth_topic, Image, depth_image_callback)
    rospy.Subscriber("/detector/box_detection", Detections, detection_callback)
    
    # Start FastAPI server with uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


if __name__ == "__main__":
    main()