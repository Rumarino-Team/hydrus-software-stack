extends RigidBody

# WebSocket connection variables
var ws := WebSocketClient.new()
var ws_connected := false
var connection_url := "ws://localhost:8000/ws"

# Submarine physical properties
export var buoyancy_factor := 1.0
export var water_drag := 0.1
export var water_angular_drag := 0.3
export var max_thrust := 100.0
export var max_torque := 20.0

# Thruster configuration
var thrusters = []
var current_cmd_vel = {"linear": Vector3.ZERO, "angular": Vector3.ZERO}

# Camera reference
onready var camera = $Camera
onready var depth_camera = $DepthCamera

# Timers for data transmission
var state_timer := 0.0
var camera_timer := 0.0
var STATE_INTERVAL := 0.05  # 20 Hz
var CAMERA_INTERVAL := 0.1  # 10 Hz

class Thruster:
	var position: Vector3  # Position relative to center of mass
	var direction: Vector3  # Thrust direction vector (normalized)
	var max_force: float
	var current_value: float = 0.0  # Current PWM value (normalized -1 to 1)

	func _init(pos: Vector3, dir: Vector3, max_f: float):
		position = pos
		direction = dir.normalized()
		max_force = max_f

	func set_value(pwm_value: float):
		# Convert PWM (-255 to 255) to normalized value (-1 to 1)
		current_value = clamp(pwm_value / 255.0, -1.0, 1.0)

	func get_force() -> Vector3:
		return direction * current_value * max_force

	func get_torque(center_of_mass: Vector3) -> Vector3:
		var force = get_force()
		var lever = position - center_of_mass
		return lever.cross(force)

func _ready():
	# Initialize physics
	set_use_custom_integrator(true)
	set_max_contacts_reported(4)
	set_contact_monitor(true)

	# Initialize thrusters (position, direction, max force)
	setup_thrusters()

	# Initialize WebSocket connection
	setup_websocket()

	# Set up camera
	if camera:
		camera.set_current(true)

	# Initial notification
	print("Submarine controller initialized")

func setup_thrusters():
	# This configuration depends on your submarine's actual thruster layout
	# Example of 8 thrusters: 4 vertical, 4 horizontal

	# Horizontal thrusters (X-configuration)
	thrusters.append(Thruster.new(Vector3(0.3, 0, 0.3), Vector3(1, 0, -1), max_thrust))
	thrusters.append(Thruster.new(Vector3(-0.3, 0, 0.3), Vector3(-1, 0, -1), max_thrust))
	thrusters.append(Thruster.new(Vector3(-0.3, 0, -0.3), Vector3(-1, 0, 1), max_thrust))
	thrusters.append(Thruster.new(Vector3(0.3, 0, -0.3), Vector3(1, 0, 1), max_thrust))

	# Vertical thrusters
	thrusters.append(Thruster.new(Vector3(0.25, 0.1, 0.25), Vector3(0, 1, 0), max_thrust))
	thrusters.append(Thruster.new(Vector3(-0.25, 0.1, 0.25), Vector3(0, 1, 0), max_thrust))
	thrusters.append(Thruster.new(Vector3(-0.25, 0.1, -0.25), Vector3(0, 1, 0), max_thrust))
	thrusters.append(Thruster.new(Vector3(0.25, 0.1, -0.25), Vector3(0, 1, 0), max_thrust))

func setup_websocket():
	# Connect WebSocket signals
	ws.connect("connection_established", self, "_on_connection_established")
	ws.connect("connection_error", self, "_on_connection_error")
	ws.connect("connection_closed", self, "_on_connection_closed")
	ws.connect("data_received", self, "_on_data_received")

	# Start connection
	var err = ws.connect_to_url(connection_url)
	if err != OK:
		print("Unable to connect to WebSocket server: ", err)
	else:
		print("Connecting to WebSocket server: " + connection_url)

func _on_connection_established(protocol):
	ws_connected = true
	print("WebSocket connection established with protocol: ", protocol)
	# Send initial log message
	send_log_message("Submarine simulation connected")

func _on_connection_error():
	ws_connected = false
	print("WebSocket connection error")

func _on_connection_closed(was_clean = false):
	ws_connected = false
	print("WebSocket connection closed, clean: ", was_clean)

	# Try to reconnect after a delay
	yield(get_tree().create_timer(2.0), "timeout")
	var err = ws.connect_to_url(connection_url)
	if err != OK:
		print("Unable to reconnect to WebSocket server: ", err)

func _on_data_received():
	# Process incoming data from ROS
	var data = ws.get_peer(1).get_packet().get_string_from_utf8()

	# Parse JSON data
	var json = JSON.parse(data)
	if json.error != OK:
		print("JSON parse error: ", json.error)
		return

	var payload = json.result

	# Handle different types of messages
	if payload.has("cmd_vel"):
		handle_cmd_vel(payload.cmd_vel)
	elif payload.has("thruster"):
		handle_thruster(payload.thruster)
	elif payload.has("depth_control"):
		handle_depth_control(payload.depth_control)
	elif payload.has("torpedo_control"):
		handle_torpedo_control(payload.torpedo_control)

func handle_cmd_vel(cmd_vel):
	# Store current command velocity
	current_cmd_vel.linear.x = cmd_vel.linear.x
	current_cmd_vel.linear.y = cmd_vel.linear.y
	current_cmd_vel.linear.z = cmd_vel.linear.z
	current_cmd_vel.angular.x = cmd_vel.angular.x
	current_cmd_vel.angular.y = cmd_vel.angular.y
	current_cmd_vel.angular.z = cmd_vel.angular.z

	# Convert cmd_vel to thruster values (simplified)
	# In a real system, this would use a thrust allocation algorithm

func handle_thruster(thruster_data):
	# Set individual thruster values
	var index = thruster_data.index - 1  # Convert 1-based to 0-based
	if index >= 0 and index < thrusters.size():
		thrusters[index].set_value(thruster_data.value)

func handle_depth_control(depth_pwm):
	# Apply depth control to vertical thrusters
	var value = float(depth_pwm)
	for i in range(4, 8):  # Assuming thrusters 4-7 are vertical
		thrusters[i].set_value(value)

func handle_torpedo_control(torpedo_pwm):
	# Not implemented: could trigger torpedo launch animation
	pass

func _integrate_forces(state):
	# Apply buoyancy
	apply_buoyancy(state)

	# Apply water drag
	apply_water_drag(state)

	# Apply thruster forces
	apply_thruster_forces(state)

func apply_buoyancy(state):
	# Simple buoyancy calculation
	var depth = -global_transform.origin.y  # Y is up in Godot
	var buoyancy_force = Vector3(0, buoyancy_factor, 0)

	# Apply more buoyancy when deeper
	if depth > 0:
		buoyancy_force.y += depth * 2.0

	state.add_central_force(buoyancy_force)

func apply_water_drag(state):
	# Apply linear drag
	var velocity = state.linear_velocity
	var drag_force = -velocity * velocity.length() * water_drag
	state.add_central_force(drag_force)

	# Apply angular drag
	var angular_velocity = state.angular_velocity
	var angular_drag = -angular_velocity * angular_velocity.length() * water_angular_drag
	state.add_torque(angular_drag)

func apply_thruster_forces(state):
	var center_of_mass = state.get_center_of_mass()

	for thruster in thrusters:
		# Apply force
		var force = thruster.get_force()
		state.add_force(force, thruster.position - center_of_mass)

func _process(delta):
	# Process WebSocket connection
	if ws_connected:
		ws.poll()

	# Send state updates at regular intervals
	state_timer += delta
	if state_timer >= STATE_INTERVAL and ws_connected:
		state_timer = 0
		send_state()

	# Send camera images at regular intervals
	camera_timer += delta
	if camera_timer >= CAMERA_INTERVAL and ws_connected:
		camera_timer = 0
		send_camera_image()
		send_depth_image()

func _physics_process(delta):
	# Apply forces from cmd_vel (simplified)
	# In real system, this would use the cmd_vel to calculate thruster values
	pass

func send_state():
	# Send submarine state to ROS
	var state = {
		"pos": [
			global_transform.origin.x,
			global_transform.origin.y,
			global_transform.origin.z
		],
		"orient": [
			global_transform.basis.get_rotation_quat().x,
			global_transform.basis.get_rotation_quat().y,
			global_transform.basis.get_rotation_quat().z,
			global_transform.basis.get_rotation_quat().w
		],
		"vel": [
			linear_velocity.x,
			linear_velocity.y,
			linear_velocity.z
		],
		"ang_vel": [
			angular_velocity.x,
			angular_velocity.y,
			angular_velocity.z
		]
	}

	var json_data = {"state": state}
	ws.get_peer(1).put_packet(JSON.print(json_data).to_utf8())

func send_camera_image():
	if !camera:
		return

	# Capture viewport texture
	var viewport = camera.get_viewport()
	var img = viewport.get_texture().get_data()
	img.flip_y() # Godot renders upside down compared to OpenCV convention

	# Convert to RGB format
	img.convert(Image.FORMAT_RGB8)

	# Encode to base64
	var base64_img = Marshalls.raw_to_base64(img.get_data())

	# Create camera data packet
	var camera_data = {
		"width": img.get_width(),
		"height": img.get_height(),
		"encoding": "rgb8",
		"data": base64_img
	}

	# Send camera data
	var json_data = {"camera": camera_data}
	ws.get_peer(1).put_packet(JSON.print(json_data).to_utf8())

func send_depth_image():
	if !depth_camera:
		return

	# This would capture depth data in a real implementation
	# For now, we'll simulate a depth image with distance data
	var width = 320
	var height = 240

	# Create fake depth data (in a real implementation, this would come from the depth camera)
	var depth_data = PoolByteArray()
	for y in height:
		for x in width:
			# Create simple depth value (increasing from left to right)
			var depth_value = 1.0 + (float(x) / width * 5.0)

			# Convert float to bytes
			var bytes = var2bytes(depth_value)
			for b in bytes:
				depth_data.append(b)

	# Encode to base64
	var base64_depth = Marshalls.raw_to_base64(depth_data)

	# Create depth data packet
	var depth_payload = {
		"width": width,
		"height": height,
		"encoding": "32FC1",
		"data": base64_depth
	}

	# Send depth data
	var json_data = {"depth": depth_payload}
	ws.get_peer(1).put_packet(JSON.print(json_data).to_utf8())

func send_log_message(message):
	# Send a log message to ROS
	var json_data = {"log": message}
	if ws_connected:
		ws.get_peer(1).put_packet(JSON.print(json_data).to_utf8())
