# systems/ThrusterSystem.gd
# Manages all thrusters as a cohesive system
class_name ThrusterSystem
extends Node

signal thruster_force_applied(total_force: Vector3, total_torque: Vector3)

var thrusters: Array[IThruster] = []
var thruster_layout: ThrusterLayout

func _init():
	thruster_layout = ThrusterLayout.new()

func add_thruster(thruster: IThruster) -> void:
	thrusters.append(thruster)
	thruster.thrust_changed.connect(_on_thruster_changed)

func get_thruster_by_id(thruster_id: int) -> IThruster:
	for thruster in thrusters:
		if thruster.id == thruster_id:
			return thruster
	return null

func set_thruster_pwm(thruster_id: int, pwm_value: float) -> void:
	var thruster = get_thruster_by_id(thruster_id)
	if thruster:
		thruster.set_pwm(pwm_value)

func update_thrusters(delta: float) -> void:
	for thruster in thrusters:
		if thruster is Thruster:
			thruster.update_pwm(delta)
		thruster.update_visual()

func apply_forces_to_body(state: PhysicsDirectBodyState3D) -> void:
	var total_force = Vector3.ZERO
	var total_torque = Vector3.ZERO

	for thruster in thrusters:
		var thrust_force = thruster.get_thrust_force()
		var world_force = state.transform.basis * thrust_force
		var local_position = thruster.position

		state.add_constant_force(world_force, local_position)
		total_force += world_force
		total_torque += local_position.cross(world_force)

	thruster_force_applied.emit(total_force, total_torque)

func execute_movement_command(command: MovementCommand) -> void:
	match command.type:
		MovementCommand.Type.FORWARD:
			thruster_layout.move_forward(self, command.intensity)
		MovementCommand.Type.BACKWARD:
			thruster_layout.move_backward(self, command.intensity)
		MovementCommand.Type.TURN_LEFT:
			thruster_layout.turn_left(self, command.intensity)
		MovementCommand.Type.TURN_RIGHT:
			thruster_layout.turn_right(self, command.intensity)
		MovementCommand.Type.MOVE_UP:
			thruster_layout.move_up(self, command.intensity)
		MovementCommand.Type.MOVE_DOWN:
			thruster_layout.move_down(self, command.intensity)
		MovementCommand.Type.STOP:
			stop_all_thrusters()

func stop_all_thrusters() -> void:
	for thruster in thrusters:
		thruster.set_pwm(1500)

func get_thruster_info() -> Array:
	var info = []
	for thruster in thrusters:
		info.append({
			"id": thruster.id,
			"current_pwm": thruster.get_current_pwm(),
			"position": thruster.position,
			"direction": thruster.direction,
			"force": thruster.get_thrust_force()
		})
	return info

func _on_thruster_changed(thruster_id: int, thrust_value: float) -> void:
	# Could be used for logging or debugging
	pass
