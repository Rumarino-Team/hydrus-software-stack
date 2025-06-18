# systems/PhysicsSystem.gd
# Handles submarine physics like buoyancy, drag, and water effects
class_name PhysicsSystem

var buoyancy_force: float = 490.0  # Force to counteract gravity
var water_drag: float = 0.5
var angular_drag: float = 0.3
var water_density: float = 1000.0  # kg/m³

func apply_physics(state: PhysicsDirectBodyState3D) -> void:
	apply_buoyancy(state)
	apply_water_drag(state)

func apply_buoyancy(state: PhysicsDirectBodyState3D) -> void:
	# Calculate depth (negative Y is deeper in Godot)
	var depth = -state.transform.origin.y

	if depth > 0:  # Only apply buoyancy when underwater
		var buoyancy = Vector3(0, buoyancy_force, 0)

		# Increase buoyancy slightly with depth to simulate water pressure
		buoyancy.y += depth * 0.1

		state.add_central_force(buoyancy)

func apply_water_drag(state: PhysicsDirectBodyState3D) -> void:
	# Linear drag - opposes movement through water
	var velocity = state.linear_velocity
	var drag_force = -velocity * velocity.length() * water_drag
	state.add_central_force(drag_force)

	# Angular drag - opposes rotation
	var angular_velocity = state.angular_velocity
	var rotational_drag = -angular_velocity * angular_velocity.length() * angular_drag
	state.add_torque(rotational_drag)

func get_physics_info() -> Dictionary:
	return {
		"buoyancy_force": buoyancy_force,
		"water_drag": water_drag,
		"angular_drag": angular_drag,
		"water_density": water_density
	}
