# systems/CameraSystem.gd
# Manages multiple cameras and switching between them
class_name CameraSystem
extends Node

signal camera_switched(old_camera: String, new_camera: String)

var cameras: Dictionary = {}
var active_camera_name: String = ""

func add_camera(name: String, camera_node: Camera3D) -> void:
	if camera_node:
		cameras[name] = camera_node
		print("📷 Added camera: ", name)

func set_active_camera(camera_name: String) -> bool:
	if not cameras.has(camera_name):
		print("❌ Camera not found: ", camera_name)
		return false

	# Deactivate all cameras
	for name in cameras:
		cameras[name].current = false

	# Activate the requested camera
	cameras[camera_name].current = true
	var old_camera = active_camera_name
	active_camera_name = camera_name

	camera_switched.emit(old_camera, camera_name)
	print("📹 Switched to camera: ", camera_name)
	return true

func get_active_camera() -> Camera3D:
	if cameras.has(active_camera_name):
		return cameras[active_camera_name]
	return null

func get_active_camera_name() -> String:
	return active_camera_name

func cycle_cameras() -> void:
	var camera_names = cameras.keys()
	if camera_names.size() == 0:
		return

	var current_index = camera_names.find(active_camera_name)
	var next_index = (current_index + 1) % camera_names.size()
	set_active_camera(camera_names[next_index])

func get_available_cameras() -> Array:
	return cameras.keys()
