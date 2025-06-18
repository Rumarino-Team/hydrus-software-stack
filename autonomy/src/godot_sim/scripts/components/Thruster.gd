# components/Thruster.gd
# Concrete implementation of a thruster component
class_name Thruster
extends IThruster

var current_pwm: float = 1500.0  # PWM value (1000-2000, 1500 is neutral)
var target_pwm: float = 1500.0
var response_time: float = 0.1
var visual_node: MeshInstance3D

func _init(thruster_id: int, pos: Vector3, dir: Vector3, max_f: float, node: MeshInstance3D = null):
	super(thruster_id, pos, dir, max_f)
	visual_node = node
	current_pwm = 1500.0
	target_pwm = 1500.0

func set_pwm(pwm_value: float) -> void:
	target_pwm = clamp(pwm_value, 1000.0, 2000.0)


func get_thrust_force() -> Vector3:
	# Convert PWM to force (-1 to 1 range)
	var force_ratio = (current_pwm - 1500.0) / 500.0
	var thrust = direction * force_ratio * max_force
	thrust_changed.emit(id, force_ratio)
	return thrust

func get_current_pwm() -> float:
	return current_pwm

func update_pwm(delta: float) -> void:
	# Gradually move current PWM towards target PWM
	var pwm_diff = target_pwm - current_pwm
	current_pwm += pwm_diff * response_time

func update_visual() -> void:
	if not visual_node:
		return

	var material = visual_node.get_surface_override_material(0)
	if not material:
		material = StandardMaterial3D.new()
		visual_node.set_surface_override_material(0, material)

	# Color intensity based on thrust
	var intensity = abs(current_pwm - 1500.0) / 500.0
	var base_color = Color(0.8, 0.2, 0.2)  # Red base

	if current_pwm > 1500.0:
		material.albedo_color = base_color.lerp(Color.YELLOW, intensity)
	elif current_pwm < 1500.0:
		material.albedo_color = base_color.lerp(Color.BLUE, intensity)
	else:
		material.albedo_color = base_color
