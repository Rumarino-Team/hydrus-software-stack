extends Spatial

# Main scene controller for submarine simulation

# References to key nodes
onready var submarine = $Submarine
onready var water_environment = $WaterEnvironment
onready var target_container = $Targets

# Target scenes for spawning
export(PackedScene) var buoy_scene
export(PackedScene) var gate_scene
export(PackedScene) var bin_scene

# Configuration properties
export var enable_websocket := true
export var enable_target_spawning := true
export var num_targets := 3
export var target_spacing := 3.0
export var simulation_name := "Hydrus Submarine Simulation"

func _ready():
    OS.set_window_title(simulation_name)
    print("Starting " + simulation_name)
    
    if enable_target_spawning:
        spawn_targets()
    
    # Connect signals
    if submarine:
        submarine.connect("target_detected", self, "_on_submarine_target_detected")

func spawn_targets():
    if not target_container or not buoy_scene:
        print("Can't spawn targets: missing target container or scene")
        return
    
    print("Spawning targets...")
    
    # Spawn buoys in a pattern
    var colors = [
        Color(1.0, 0.0, 0.0),  # Red
        Color(0.0, 1.0, 0.0),  # Green
        Color(0.0, 0.0, 1.0),  # Blue
        Color(1.0, 1.0, 0.0),  # Yellow
        Color(1.0, 0.0, 1.0)   # Purple
    ]
    
    # Create a grid of targets
    var grid_size = ceil(sqrt(num_targets))
    var index = 0
    
    for i in range(grid_size):
        for j in range(grid_size):
            if index >= num_targets:
                break
                
            var position = Vector3(
                (i - grid_size/2) * target_spacing,
                -1.0 - randf() * 2.0,
                (j - grid_size/2) * target_spacing
            )
            
            # Create the target
            var target = buoy_scene.instance()
            target_container.add_child(target)
            
            # Configure the target
            target.global_transform.origin = position
            target.target_color = colors[index % colors.size()]
            target.target_id = index
            target.is_moving = (randf() > 0.5)
            
            index += 1
    
    print("Spawned " + str(index) + " targets")

func _on_submarine_target_detected(target):
    # Called when submarine detects a target
    print("Submarine detected target: " + str(target.get_detection_info()))

func _process(_delta):
    # Update UI if needed
    pass

func _input(event):
    # Handle key events
    if event is InputEventKey and event.pressed:
        match event.scancode:
            KEY_ESCAPE:
                # Quit simulation
                get_tree().quit()
            KEY_R:
                # Reset submarine position
                if submarine:
                    submarine.global_transform.origin = Vector3(0, -2, 0)
                    submarine.rotation = Vector3.ZERO
                    submarine.linear_velocity = Vector3.ZERO
                    submarine.angular_velocity = Vector3.ZERO