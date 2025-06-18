extends RigidBody3D

# Submarine simulation with modular architecture
# This is the main controller that orchestrates all submarine systems

# === SYSTEMS ===
var thruster_system: ThrusterSystem
var camera_system: CameraSystem
var physics_system: PhysicsSystem
var input_handler: InputHandler

# === CONFIGURATION ===
@export var submarine_mass := 50.0
@export var max_thrust_force := 100.0

# === SCENE REFERENCES ===
@onready var thruster_nodes = [
	$Thrusters/Thruster1, $Thrusters/Thruster2, $Thrusters/Thruster3, $Thrusters/Thruster4,
	$Thrusters/Thruster5, $Thrusters/Thruster6, $Thrusters/Thruster7, $Thrusters/Thruster8
]

@onready var camera_nodes = {
	"main": $Camera,
	"forward": $CameraGimbal/ForwardCamera,
	"depth": $CameraGimbal/DepthCamera
}

func _ready():
	print("🚀 Initializing Submarine Simulation...")

	# Set up physics
	mass = submarine_mass

	# Initialize all systems
	setup_thruster_system()
	setup_camera_system()
	setup_physics_system()
	setup_input_handler()

	print("✅ Submarine simulation ready with %d thrusters" % thruster_system.thrusters.size())

func setup_thruster_system():
	print("🔧 Setting up thruster system...")
	thruster_system = ThrusterSystem.new()
	add_child(thruster_system)

	# Create thrusters based on scene layout
	var thruster_configs = get_thruster_configurations()

	for i in range(thruster_configs.size()):
		if i < thruster_nodes.size() and thruster_nodes[i]:
			var config = thruster_configs[i]
			var thruster = Thruster.new(
				config.id,
				config.position,
				config.direction,
				max_thrust_force,
				thruster_nodes[i]
			)
			thruster_system.add_thruster(thruster)

func setup_camera_system():
	print("📷 Setting up camera system...")
	camera_system = CameraSystem.new()
	add_child(camera_system)

	for camera_name in camera_nodes:
		camera_system.add_camera(camera_name, camera_nodes[camera_name])

	camera_system.set_active_camera("forward")

func setup_physics_system():
	print("⚖️ Setting up physics system...")
	physics_system = PhysicsSystem.new()
	physics_system.buoyancy_force = submarine_mass * 9.8  # Counteract gravity
	physics_system.water_drag = 0.5
	physics_system.angular_drag = 0.3

func setup_input_handler():
	print("🎮 Setting up input handler...")
	input_handler = InputHandler.new()
	add_child(input_handler)

	# Connect input signals to submarine actions
	input_handler.movement_requested.connect(_on_movement_requested)
	input_handler.camera_switch_requested.connect(_on_camera_switch_requested)

func get_thruster_configurations() -> Array:
	# Define thruster positions and directions matching the scene
	return [
		{"id": 1, "position": Vector3(-0.7, 0, -0.48), "direction": Vector3(-1, 0, -1)},
		{"id": 2, "position": Vector3(-0.3, 0, -0.48), "direction": Vector3(0, 1, 0)},
		{"id": 3, "position": Vector3(0.3, 0, -0.48), "direction": Vector3(-1, 0, 0)},
		{"id": 4, "position": Vector3(0.7, 0, -0.48), "direction": Vector3(1, 0, -1)},
		{"id": 5, "position": Vector3(-0.7, 0, 0.48), "direction": Vector3(-1, 0, 1)},
		{"id": 6, "position": Vector3(0.3, 0, 0.48), "direction": Vector3(1, 0, 0)},
		{"id": 7, "position": Vector3(-0.3, 0, 0.48), "direction": Vector3(0, 1, 0)},
		{"id": 8, "position": Vector3(0.7, 0, 0.48), "direction": Vector3(1, 0, 1)}
	]

# === PHYSICS INTEGRATION ===
func _integrate_forces(state):
	# Apply physics through the physics system
	physics_system.apply_physics(state)

	# Apply thruster forces
	thruster_system.apply_forces_to_body(state)

func _process(delta):
	# Update all systems
	thruster_system.update_thrusters(delta)
	input_handler.process_input()

# === EVENT HANDLERS ===
func _on_movement_requested(command: MovementCommand):
	print("🎯 Executing movement: ", MovementCommand.Type.keys()[command.type])
	thruster_system.execute_movement_command(command)

func _on_camera_switch_requested(camera_name: String):
	if camera_name == "cycle":
		camera_system.cycle_cameras()
	else:
		print("📹 Switching to camera: ", camera_name)
		camera_system.set_active_camera(camera_name)

# === PUBLIC API ===
func get_submarine_status() -> Dictionary:
	return {
		"position": global_transform.origin,
		"rotation": global_transform.basis.get_euler(),
		"velocity": linear_velocity,
		"active_camera": camera_system.get_active_camera_name(),
		"thrusters": thruster_system.get_thruster_info()
	}
