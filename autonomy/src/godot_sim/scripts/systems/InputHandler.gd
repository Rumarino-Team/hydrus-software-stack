# systems/InputHandler.gd
# Handles keyboard input and translates it to movement commands
class_name InputHandler
extends Node

signal movement_requested(command: MovementCommand)
signal camera_switch_requested(camera_name: String)

var movement_intensity: float = 1.0

func process_input() -> void:
	# Movement controls
	if Input.is_action_pressed("ui_up"):
		movement_requested.emit(MovementCommand.forward(movement_intensity))
	elif Input.is_action_pressed("ui_down"):
		movement_requested.emit(MovementCommand.backward(movement_intensity))
	elif Input.is_action_pressed("ui_left"):
		movement_requested.emit(MovementCommand.turn_left(movement_intensity))
	elif Input.is_action_pressed("ui_right"):
		movement_requested.emit(MovementCommand.turn_right(movement_intensity))
	elif Input.is_action_pressed("ui_accept"):  # Enter key
		movement_requested.emit(MovementCommand.move_up(movement_intensity))
	elif Input.is_action_pressed("ui_cancel"):  # Escape key
		movement_requested.emit(MovementCommand.move_down(movement_intensity))
	else:
		# Stop all movement when no keys are pressed
		movement_requested.emit(MovementCommand.stop())

	# Camera switching (on key press, not hold)
	if Input.is_action_just_pressed("ui_select"):  # Space key
		camera_switch_requested.emit("cycle")
	elif Input.is_action_just_pressed("key_1"):
		camera_switch_requested.emit("main")
	elif Input.is_action_just_pressed("key_2"):
		camera_switch_requested.emit("forward")
	elif Input.is_action_just_pressed("key_3"):
		camera_switch_requested.emit("depth")

func set_movement_intensity(intensity: float) -> void:
	movement_intensity = clamp(intensity, 0.0, 1.0)
