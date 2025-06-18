extends RigidBody3D

# Submarine physical properties
@export var submarine_mass := 50.0
@export var water_drag := 0.5
@export var angular_drag := 0.3
@export var buoyancy_force := 490.0  # Force to counteract gravity (50kg * 9.8 m/s^2)

# Visual properties
@export var submarine_color := Color(0.2, 0.2, 0.8, 1.0)  # Blue submarine
@export var thruster_color := Color(0.8, 0.2, 0.2, 1.0)   # Red thrusters

# Submarine dimensions
@export var submarine_length := 2.0
@export var submarine_width := 0.8
@export var submarine_height := 0.6

# Thruster properties
@export var thruster_size := 0.1
@export var max_thrust_force := 100.0
@export var thruster_response_time := 0.1

# References to visual components
var submarine_body: MeshInstance3D
var thruster_nodes := []
var collision_shape: CollisionShape3D

# Thruster configuration - matches the ASCII diagram from the codebase
var thrusters := []

class Thruster:
	var node: MeshInstance3D
	var position: Vector3
	var direction: Vector3
	var current_pwm: float = 1500.0  # PWM value (1000-2000, 1500 is neutral)
	var target_pwm: float = 1500.0
	var max_force: float
	var id: int

	func _init(thruster_id: int, pos: Vector3, dir: Vector3, force: float):
		id = thruster_id
		position = pos
		direction = dir.normalized()
		max_force = force
		current_pwm = 1500.0
		target_pwm = 1500.0

	func set_pwm(pwm_value: float):
		target_pwm = clamp(pwm_value, 1000.0, 2000.0)

	func get_thrust_force() -> Vector3:
		# Convert PWM to force (-1 to 1 range)
		var force_ratio = (current_pwm - 1500.0) / 500.0
		return direction * force_ratio * max_force

	func update_visual():
		if node:
			# Change thruster color based on activity
			var material = node.get_surface_override_material(0)
			if not material:
				material = StandardMaterial3D.new()
				node.set_surface_override_material(0, material)

			# Color intensity based on thrust
			var intensity = abs(current_pwm - 1500.0) / 500.0
			var base_color = Color(0.8, 0.2, 0.2)  # Red base
			if current_pwm > 1500.0:
				material.albedo_color = base_color.lerp(Color.YELLOW, intensity)
			elif current_pwm < 1500.0:
				material.albedo_color = base_color.lerp(Color.BLUE, intensity)
			else:
				material.albedo_color = base_color

# Camera references
@onready var main_camera = $Camera
@onready var forward_camera = $CameraGimbal/ForwardCamera
@onready var depth_camera = $CameraGimbal/DepthCamera

# Current active camera
var current_camera_index = 1  # 0=main, 1=forward, 2=depth
var cameras = []

func _ready():
	# Set up physics properties
	mass = submarine_mass

	# Create visual components
	create_submarine_body()
	create_collision_shape()
	setup_thrusters()

	# Connect to ROS thruster commands
	setup_ros_connections()

	# Setup camera array
	cameras = [main_camera, forward_camera, depth_camera]

	# Set initial active camera
	switch_to_camera(current_camera_index)

	print("Submarine simulation initialized with ", thrusters.size(), " thrusters")

func create_submarine_body():
	# Create the main submarine body (rectangular mesh)
	submarine_body = MeshInstance3D.new()
	submarine_body.name = "SubmarineBody"
	add_child(submarine_body)

	# Create rectangular box mesh for submarine
	var box_mesh = BoxMesh.new()
	box_mesh.size = Vector3(submarine_length, submarine_height, submarine_width)
	submarine_body.mesh = box_mesh

	# Create material for the submarine body
	var material = StandardMaterial3D.new()
	material.albedo_color = submarine_color
	material.metallic = 0.8
	material.roughness = 0.3
	submarine_body.set_surface_override_material(0, material)

	print("Created rectangular submarine mesh with dimensions: ", submarine_length, "x", submarine_height, "x", submarine_width)

func create_collision_shape():
	# Create collision shape that matches the rectangular mesh
	collision_shape = CollisionShape3D.new()
	collision_shape.name = "CollisionShape"
	add_child(collision_shape)

	var box_shape = BoxShape3D.new()
	box_shape.size = Vector3(submarine_length, submarine_height, submarine_width)
	collision_shape.shape = box_shape

func setup_thrusters():
	# Based on the ASCII diagram in controllers.py:
	#  1 *            * 5
	#     \          /
	#      |________|
	#  2*--|        |--* 6
	#      |        |
	#      |        |
	#  3*--|________|--* 7
	#      |        |
	#  4 */          \* 8

	var half_length = submarine_length / 2
	var half_width = submarine_width / 2
	var half_height = submarine_height / 2

	# Thruster 1 - Front left, angled
	create_thruster(1, Vector3(-half_length * 0.7, 0, -half_width * 1.2), Vector3(-1, 0, -1))

	# Thruster 2 - Left side, depth control
	create_thruster(2, Vector3(-half_length * 0.3, 0, -half_width * 1.2), Vector3(0, 1, 0))

	# Thruster 3 - Left side, torpedo thruster
	create_thruster(3, Vector3(half_length * 0.3, 0, -half_width * 1.2), Vector3(-1, 0, 0))

	# Thruster 4 - Back left, angled
	create_thruster(4, Vector3(half_length * 0.7, 0, -half_width * 1.2), Vector3(1, 0, -1))

	# Thruster 5 - Front right, angled
	create_thruster(5, Vector3(-half_length * 0.7, 0, half_width * 1.2), Vector3(-1, 0, 1))

	# Thruster 6 - Right side, torpedo thruster
	create_thruster(6, Vector3(half_length * 0.3, 0, half_width * 1.2), Vector3(1, 0, 0))

	# Thruster 7 - Right side, depth control
	create_thruster(7, Vector3(-half_length * 0.3, 0, half_width * 1.2), Vector3(0, 1, 0))

	# Thruster 8 - Back right, angled
	create_thruster(8, Vector3(half_length * 0.7, 0, half_width * 1.2), Vector3(1, 0, 1))

func create_thruster(id: int, position: Vector3, direction: Vector3):
	# Create visual representation
	var thruster_visual = MeshInstance3D.new()
	thruster_visual.name = "Thruster_" + str(id)
	add_child(thruster_visual)

	# Create cube mesh for thruster
	var cube_mesh = BoxMesh.new()
	cube_mesh.size = Vector3(thruster_size, thruster_size, thruster_size)
	thruster_visual.mesh = cube_mesh

	# Position the thruster
	thruster_visual.position = position

	# Create material
	var material = StandardMaterial3D.new()
	material.albedo_color = thruster_color
	thruster_visual.set_surface_override_material(0, material)

	# Create thruster object
	var thruster = Thruster.new(id, position, direction, max_thrust_force)
	thruster.node = thruster_visual
	thrusters.append(thruster)
	thruster_nodes.append(thruster_visual)

	# Add a small label above each thruster for identification
	create_thruster_label(thruster_visual, str(id), position)

func create_thruster_label(parent: MeshInstance3D, text: String, pos: Vector3):
	# Create a 3D label for the thruster
	var label = Label3D.new()
	label.text = text
	label.pixel_size = 0.01
	label.position = Vector3(0, thruster_size + 0.05, 0)
	parent.add_child(label)

func setup_ros_connections():
	# This would connect to ROS topics in a real implementation
	# For now, we'll simulate with keyboard input or external signals
	pass

func set_thruster_pwm(thruster_id: int, pwm_value: float):
	"""Set PWM value for a specific thruster"""
	for thruster in thrusters:
		if thruster.id == thruster_id:
			thruster.set_pwm(pwm_value)
			break

func _integrate_forces(state):
	# Apply buoyancy to counteract gravity
	state.add_constant_central_force(Vector3(0, buoyancy_force, 0))

	# Apply water drag
	apply_drag(state)

	# Apply thruster forces
	apply_thruster_forces(state)

func apply_drag(state):
	# Linear drag
	var velocity = state.linear_velocity
	var drag_force = -velocity * velocity.length() * water_drag
	state.add_constant_central_force(drag_force)

	# Angular drag
	var angular_velocity = state.angular_velocity
	var angular_drag_force = -angular_velocity * angular_velocity.length() * angular_drag
	state.add_constant_torque(angular_drag_force)

func apply_thruster_forces(state):
	for thruster in thrusters:
		# Update thruster PWM gradually (simulate response time)
		var pwm_diff = thruster.target_pwm - thruster.current_pwm
		thruster.current_pwm += pwm_diff * thruster_response_time

		# Get thrust force in world coordinates
		var local_force = thruster.get_thrust_force()
		var world_force = global_transform.basis * local_force

		# Apply force at thruster position
		var local_position = thruster.position
		state.add_constant_force(world_force, local_position)

		# Update visual representation
		thruster.update_visual()

func _process(delta):
	# Handle keyboard input for testing
	handle_debug_input()

	# Update thruster visuals
	update_thruster_visuals()

func switch_to_camera(camera_index: int):
	"""Switch to a specific camera by index"""
	if camera_index < 0 or camera_index >= cameras.size():
		return

	# Deactivate all cameras
	for camera in cameras:
		if camera:
			camera.current = false

	# Activate the selected camera
	if cameras[camera_index]:
		cameras[camera_index].current = true
		current_camera_index = camera_index
		print("Switched to camera: ", camera_index)

func get_active_camera() -> Camera3D:
	"""Get the currently active camera"""
	return cameras[current_camera_index]

func cycle_cameras():
	"""Cycle through all available cameras"""
	var next_index = (current_camera_index + 1) % cameras.size()
	switch_to_camera(next_index)

func handle_debug_input():
	# Debug controls for testing thrusters
	if Input.is_action_pressed("ui_up"):
		# Forward movement - thrusters 1, 4, 5, 8
		set_thruster_pwm(1, 1600)
		set_thruster_pwm(4, 1600)
		set_thruster_pwm(5, 1600)
		set_thruster_pwm(8, 1600)
	elif Input.is_action_pressed("ui_down"):
		# Backward movement
		set_thruster_pwm(1, 1400)
		set_thruster_pwm(4, 1400)
		set_thruster_pwm(5, 1400)
		set_thruster_pwm(8, 1400)
	elif Input.is_action_pressed("ui_left"):
		# Turn left
		set_thruster_pwm(1, 1400)
		set_thruster_pwm(4, 1400)
		set_thruster_pwm(5, 1600)
		set_thruster_pwm(8, 1600)
	elif Input.is_action_pressed("ui_right"):
		# Turn right
		set_thruster_pwm(1, 1600)
		set_thruster_pwm(4, 1600)
		set_thruster_pwm(5, 1400)
		set_thruster_pwm(8, 1400)
	elif Input.is_action_pressed("ui_accept"):
		# Up movement - depth thrusters 2, 7
		set_thruster_pwm(2, 1600)
		set_thruster_pwm(7, 1600)
	elif Input.is_action_pressed("ui_cancel"):
		# Down movement
		set_thruster_pwm(2, 1400)
		set_thruster_pwm(7, 1400)
	elif Input.is_action_just_pressed("ui_select"):  # Space key
		cycle_cameras()
	elif Input.is_key_pressed(KEY_1):
		switch_to_camera(0)  # Main camera
	elif Input.is_key_pressed(KEY_2):
		switch_to_camera(1)  # Forward camera
	elif Input.is_key_pressed(KEY_3):
		switch_to_camera(2)  # Depth camera
	else:
		# Reset all thrusters to neutral
		for thruster in thrusters:
			thruster.set_pwm(1500)

func update_thruster_visuals():
	# Update thruster visual indicators
	for thruster in thrusters:
		thruster.update_visual()

func get_thruster_info() -> Array:
	"""Get current state of all thrusters for debugging"""
	var info = []
	for thruster in thrusters:
		info.append({
			"id": thruster.id,
			"pwm": thruster.current_pwm,
			"target_pwm": thruster.target_pwm,
			"force": thruster.get_thrust_force(),
			"position": thruster.position
		})
	return info

# External API functions for ROS integration
func handle_thruster_command(thruster_id: int, pwm_value: float):
	"""Handle thruster commands from ROS"""
	set_thruster_pwm(thruster_id, pwm_value)

func handle_depth_command(pwm_value: float):
	"""Handle depth control commands from ROS"""
	# Depth thrusters are 2 and 7
	set_thruster_pwm(2, pwm_value)
	set_thruster_pwm(7, pwm_value)

func handle_torpedo_command(pwm_value: float):
	"""Handle torpedo thruster commands from ROS"""
	# Torpedo thrusters are 3 and 6
	set_thruster_pwm(3, pwm_value)
	set_thruster_pwm(6, pwm_value)
