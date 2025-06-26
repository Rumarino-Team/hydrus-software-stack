# Modular Tmux Session Manager

The Hydrus tmux session manager has been refactored to use a dictionary-based configuration system, making it highly modular and customizable.

## Features

- **Dictionary-based configuration**: Each window is defined by a configuration dictionary
- **Flexible layouts**: Support for different tmux layouts per window
- **Command-line interface**: Various options for managing configurations
- **Configuration validation**: Ensure window configurations are properly structured
- **Selective window creation**: Create only specific windows
- **External configuration files**: Load configurations from JSON files

## Configuration Structure

Each window is defined by a dictionary with the following structure:

```python
{
    "window_name": {
        "window_index": 0,              # Unique index for the window
        "layout": "even-horizontal",    # Tmux layout (optional)
        "panes": [
            {
                "name": "Pane Name",    # Descriptive name
                "command": "echo 'Hello'", # Command to run
                "split": None           # No split for first pane
            },
            {
                "name": "Second Pane",
                "command": "htop",
                "split": "horizontal"   # Split direction: "horizontal" or "vertical"
            }
        ]
    }
}
```

### Required Fields

- `window_index`: Unique integer identifying the window order
- `panes`: List of pane configurations
- `panes[].command`: Command to execute in the pane
- `panes[].split`: Split direction for panes after the first (None for first pane)

### Optional Fields

- `layout`: Tmux layout name (default: "even-horizontal")
- `panes[].name`: Descriptive name for the pane (for documentation)

## Usage Examples

### Basic Usage
```bash
# Create all configured windows
python3 start_tmux_sessions.py

# Create only specific windows
python3 start_tmux_sessions.py --windows "Controls" "Arduino"
```

### Configuration Management
```bash
# List all configured windows
python3 start_tmux_sessions.py --list-windows

# Validate current configuration
python3 start_tmux_sessions.py --validate

# Save current configuration to file
python3 start_tmux_sessions.py --save-config my_config.json

# Load configuration from file
python3 start_tmux_sessions.py --config config/tmux_windows.json
```

### Development Workflow
```bash
# Create only development and monitoring windows
python3 start_tmux_sessions.py --windows "Development" "Arduino"

# Load custom configuration for testing
python3 start_tmux_sessions.py --config test_config.json --windows "Controls"
```

## Available Layouts

- `even-horizontal`: Panes evenly distributed horizontally
- `even-vertical`: Panes evenly distributed vertically
- `main-horizontal`: Large main pane on top, others below
- `main-vertical`: Large main pane on left, others on right
- `tiled`: Panes arranged in a tiled pattern

## Pane Split Directions

- `"horizontal"`: Split horizontally (new pane below)
- `"vertical"`: Split vertically (new pane to the right)
- `null`/`None`: No split (only for first pane)

## Example Configurations

### Simple Two-Pane Window
```json
{
  "Simple": {
    "window_index": 0,
    "layout": "even-horizontal",
    "panes": [
      {
        "name": "Main Terminal",
        "command": "bash",
        "split": null
      },
      {
        "name": "System Monitor",
        "command": "htop",
        "split": "horizontal"
      }
    ]
  }
}
```

### Complex Four-Pane Development Window
```json
{
  "Development": {
    "window_index": 1,
    "layout": "tiled",
    "panes": [
      {
        "name": "Code Editor",
        "command": "cd /workspace && vim",
        "split": null
      },
      {
        "name": "Build Output",
        "command": "cd /workspace && tail -f build.log",
        "split": "horizontal"
      },
      {
        "name": "Git Status",
        "command": "cd /workspace && watch git status",
        "split": "vertical"
      },
      {
        "name": "Test Runner",
        "command": "cd /workspace && pytest --watch",
        "split": "horizontal"
      }
    ]
  }
}
```

## Programming Interface

You can also modify configurations programmatically:

```python
from start_tmux_sessions import HydrusTmuxManager

manager = HydrusTmuxManager()

# Add a new window configuration
new_window = {
    "window_index": 5,
    "layout": "main-vertical",
    "panes": [
        {
            "name": "Main Task",
            "command": "python3 my_script.py",
            "split": None
        },
        {
            "name": "Monitor",
            "command": "watch ps aux",
            "split": "horizontal"
        }
    ]
}

manager.add_window_config("Custom Window", new_window)

# Validate the configuration
if manager.validate_window_config(new_window):
    print("Configuration is valid!")

# Create the session
manager.main()
```

## Migration from Old System

The old individual window creation methods have been replaced with the modular system. If you need to customize windows:

1. **Export current config**: `python3 start_tmux_sessions.py --save-config current.json`
2. **Edit the JSON file** with your customizations
3. **Load the config**: `python3 start_tmux_sessions.py --config current.json`

## Troubleshooting

### Configuration Validation Errors
```bash
# Check what's wrong with your configuration
python3 start_tmux_sessions.py --validate
```

Common issues:
- First pane has a split defined (should be `null`)
- Missing required fields (`command`, `split` for non-first panes)
- Invalid split directions (must be "horizontal" or "vertical")
- Duplicate window indices

### Window Creation Failures
- Check that all script paths exist
- Verify ROS environment is properly sourced
- Ensure required dependencies are installed

### Tmux Issues
- Make sure tmux is installed: `sudo apt install tmux`
- Check existing sessions: `tmux list-sessions`
- Kill problematic sessions: `tmux kill-session -t hydrus`

This modular system provides much greater flexibility while maintaining compatibility with the existing Hydrus infrastructure.
