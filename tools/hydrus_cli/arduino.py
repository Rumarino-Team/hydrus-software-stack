import json
import subprocess
import sys
import time
from pathlib import Path

import typer

# Add the project root to the Python path
project_root = Path(__file__).parent.parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

try:
    from devices.Arduino.arduino import Arduino
    from devices.Arduino.virtual_arduino import check_and_create_virtual_arduino
except ImportError as e:
    typer.echo(f"Error importing Arduino modules: {e}")
    typer.echo("Please ensure you are running from the project root directory.")
    typer.echo(f"Expected project root: {project_root}")
    raise typer.Exit(1)

arduino_command = typer.Typer()

# Board nametags for easy selection
BOARD_TYPES = {
    "uno": "arduino:avr:uno",
    "nano": "arduino:avr:nano",
    "mega": "arduino:avr:mega",
    "leonardo": "arduino:avr:leonardo",
    "micro": "arduino:avr:micro",
    "esp32": "esp32:esp32:esp32",
    "esp8266": "esp8266:esp8266:nodemcuv2",
    "teensy40": "teensy:avr:teensy40",
    "teensy41": "teensy:avr:teensy41",
}


def find_arduino_sketch_dir():
    """Find the Arduino sketch directory in the project."""
    # Look for Arduino sketches in common locations
    project_root = Path(__file__).parent.parent.parent

    # Default location for Hydrus Arduino code
    default_sketch = project_root / "devices" / "Arduino" / "HydrusModule"
    if default_sketch.exists():
        ino_files = list(default_sketch.glob("*.ino"))
        if ino_files:
            return default_sketch


# Default sketch path
DEFAULT_SKETCH_PATH = Path("devices/Arduino/HydrusModule")


def get_board_fqbn(board_nametag: str) -> str:
    """Convert board nametag to FQBN (Fully Qualified Board Name)"""
    return BOARD_TYPES.get(board_nametag, board_nametag)


def auto_detect_board() -> str:
    """Auto-detect connected Arduino board"""
    try:
        result = subprocess.run(
            ["arduino-cli", "board", "list", "--format", "json"],
            capture_output=True,
            text=True,
            check=True,
        )
        boards_data = json.loads(result.stdout)

        # Check if there are detected ports with matching boards
        if "detected_ports" in boards_data:
            for port_info in boards_data["detected_ports"]:
                if port_info.get("matching_boards"):
                    return port_info["matching_boards"][0]["fqbn"]

        typer.echo(
            "No Arduino board detected. Please connect your board and try again."
        )
        raise typer.Exit(1)
    except subprocess.CalledProcessError as e:
        typer.echo(f"Error detecting board: {e}")
        raise typer.Exit(1)
    except json.JSONDecodeError:
        typer.echo("Error parsing board detection output")
        raise typer.Exit(1)


def get_sketch_path() -> Path:
    """Get the default sketch path, ensuring it exists"""
    sketch_path = DEFAULT_SKETCH_PATH
    if not sketch_path.exists():
        typer.echo(f"Default sketch path not found: {sketch_path}")
        typer.echo("Please ensure you're running from the project root directory.")
        raise typer.Exit(1)
    return sketch_path


def check_arduino_cli():
    """Check if arduino-cli is installed and available."""
    try:
        subprocess.run(
            ["arduino-cli", "version"], capture_output=True, text=True, check=True
        )
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


@arduino_command.command()
def compile(
    board: str = typer.Argument(
        "uno",
        help="Board nametag: uno, nano, mega, leonardo, micro, esp32, esp8266, teensy40, teensy41",
    )
):
    """Compile Arduino sketch for the specified board (uses default HydrusModule sketch)."""
    # Check if arduino-cli is available
    if not check_arduino_cli():
        typer.echo(
            "❌ arduino-cli not found. Please install arduino-cli first.", err=True
        )
        typer.echo(
            "   Install: https://arduino.github.io/arduino-cli/0.35/installation/"
        )
        raise typer.Exit(1)

    # Find sketch directory (always use default)
    sketch_dir = find_arduino_sketch_dir()

    if not sketch_dir or not sketch_dir.exists():
        typer.echo("❌ Arduino sketch directory not found.", err=True)
        typer.echo("   Expected: devices/Arduino/HydrusModule/")
        raise typer.Exit(1)

    # Find .ino file in the directory
    ino_files = list(sketch_dir.glob("*.ino"))
    if not ino_files:
        typer.echo(f"❌ No .ino files found in {sketch_dir}", err=True)
        raise typer.Exit(1)

    sketch_file = ino_files[0]  # Use the first .ino file found
    board_fqbn = get_board_fqbn(board)

    typer.echo(f"🔨 Compiling: {sketch_file.name}")
    typer.echo(f"📋 Board: {board} ({board_fqbn})")

    # Prepare arduino-cli compile command
    cmd = ["arduino-cli", "compile", "--fqbn", board_fqbn, str(sketch_dir)]

    try:
        # Run compilation
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode == 0:
            typer.echo("✅ Compilation successful!")
        else:
            typer.echo("❌ Compilation failed!", err=True)
            if result.stderr:
                typer.echo(f"Error: {result.stderr}", err=True)
            raise typer.Exit(1)

    except subprocess.CalledProcessError as e:
        typer.echo(f"❌ Compilation failed: {e}", err=True)
        raise typer.Exit(1)


@arduino_command.command()
def upload(
    board: str = typer.Option(
        None, "--board", "-b", help="Override board detection (e.g., uno, nano, esp32)"
    ),
    port: str = typer.Option(
        None, "--port", "-p", help="Override port detection (e.g., /dev/ttyACM0)"
    ),
):
    """Upload Arduino sketch to connected board (auto-detects board type and port)."""

    # Check if arduino-cli is available
    if not check_arduino_cli():
        typer.echo(
            "❌ arduino-cli not found. Please install arduino-cli first.", err=True
        )
        typer.echo(
            "   Install: https://arduino.github.io/arduino-cli/0.35/installation/"
        )
        raise typer.Exit(1)

    # Find sketch directory (always use default)
    sketch_dir = find_arduino_sketch_dir()

    if not sketch_dir or not sketch_dir.exists():
        typer.echo("❌ Arduino sketch directory not found.", err=True)
        typer.echo("   Expected: devices/Arduino/HydrusModule/")
        raise typer.Exit(1)

    # Find .ino file in the directory
    ino_files = list(sketch_dir.glob("*.ino"))
    if not ino_files:
        typer.echo(f"❌ No .ino files found in {sketch_dir}", err=True)
        raise typer.Exit(1)

    sketch_file = ino_files[0]  # Use the first .ino file found

    typer.echo(f"⬆️  Uploading: {sketch_file.name}")

    # Auto-detect board and port (or use manual overrides)
    detected_board = None
    detected_port = None

    if board and port:
        # Use manual overrides
        detected_board = get_board_fqbn(board)
        detected_port = port
        typer.echo(f"📋 Using manual board: {board} ({detected_board})")
        typer.echo(f"📡 Using manual port: {detected_port}")
    else:
        # Auto-detect
        typer.echo("🔍 Auto-detecting board and port...")
        try:
            detect_result = subprocess.run(
                ["arduino-cli", "board", "list", "--format", "json"],
                capture_output=True,
                text=True,
                check=True,
            )

            boards_data = json.loads(detect_result.stdout)

            # Check if there are detected ports with matching boards
            if "detected_ports" in boards_data:
                for port_info in boards_data["detected_ports"]:
                    if port_info.get("matching_boards"):
                        detected_board = port_info["matching_boards"][0]["fqbn"]
                        detected_port = port_info["port"]["address"]
                        break

            if not detected_board or not detected_port:
                # No Arduino board detected - provide helpful error message
                if "detected_ports" in boards_data and boards_data["detected_ports"]:
                    available_ports = [
                        p["port"]["address"] for p in boards_data["detected_ports"]
                    ]
                    typer.echo("❌ No Arduino board detected on available ports.")
                    typer.echo(f"   Available ports: {', '.join(available_ports)}")
                    typer.echo(
                        "   Try using manual override: --board uno --port /dev/ttyACM0"
                    )
                else:
                    typer.echo(
                        "❌ No serial ports detected. Please connect your Arduino board."
                    )
                raise typer.Exit(1)

            typer.echo(f"✅ Detected board: {detected_board}")
            typer.echo(f"📡 Using port: {detected_port}")

        except (subprocess.CalledProcessError, json.JSONDecodeError, KeyError) as e:
            typer.echo("❌ Board detection failed. Please check your board connection.")
            typer.echo(f"   Error: {e}")
            typer.echo("   Try using manual override: --board uno --port /dev/ttyACM0")
            raise typer.Exit(1)

    # Compile first
    typer.echo("🔨 Compiling...")
    try:
        compile_cmd = [
            "arduino-cli",
            "compile",
            "--fqbn",
            detected_board,
            str(sketch_dir),
        ]
        result = subprocess.run(compile_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            typer.echo("❌ Compilation failed!", err=True)
            typer.echo(f"Error: {result.stderr}", err=True)
            raise typer.Exit(1)
        typer.echo("✅ Compilation successful!")
    except subprocess.CalledProcessError as e:
        typer.echo(f"❌ Compilation failed: {e}", err=True)
        raise typer.Exit(1)

    # Upload
    cmd = [
        "arduino-cli",
        "upload",
        "--fqbn",
        detected_board,
        "--port",
        detected_port,
        str(sketch_dir),
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode == 0:
            typer.echo("✅ Upload successful!")
            typer.echo(f"🎉 Sketch uploaded to {detected_port}")
        else:
            typer.echo("❌ Upload failed!", err=True)
            if result.stderr:
                typer.echo(f"Error: {result.stderr}", err=True)
            raise typer.Exit(1)

    except subprocess.CalledProcessError as e:
        typer.echo(f"❌ Upload failed: {e}", err=True)
        raise typer.Exit(1)


@arduino_command.command()
def list_boards():
    """List available Arduino board nametags."""
    typer.echo("📋 Available board nametags:")
    for nametag, fqbn in BOARD_TYPES.items():
        typer.echo(f"  {nametag:<10} -> {fqbn}")

    typer.echo("\n� To see all installed boards:")
    if check_arduino_cli():
        try:
            subprocess.run(["arduino-cli", "board", "listall"], check=True)
        except subprocess.CalledProcessError as e:
            typer.echo(f"❌ Failed to list boards: {e}", err=True)
    else:
        typer.echo("❌ arduino-cli not found")


@arduino_command.command()
def list_ports():
    """List available serial ports and connected boards."""
    if not check_arduino_cli():
        typer.echo(
            "❌ arduino-cli not found. Please install arduino-cli first.", err=True
        )
        raise typer.Exit(1)

    typer.echo("🔌 Available serial ports:")
    try:
        subprocess.run(["arduino-cli", "board", "list"], check=True)
    except subprocess.CalledProcessError as e:
        typer.echo(f"❌ Failed to list ports: {e}", err=True)


@arduino_command.command()
def install_core(
    core: str = typer.Argument(
        help="Core to install (e.g., arduino:avr, arduino:samd)"
    ),
):
    """Install Arduino core."""
    if not check_arduino_cli():
        typer.echo(
            "❌ arduino-cli not found. Please install arduino-cli first.", err=True
        )
        raise typer.Exit(1)

    typer.echo(f"📦 Installing Arduino core: {core}")
    try:
        subprocess.run(["arduino-cli", "core", "install", core], check=True)
        typer.echo(f"✅ Successfully installed {core}")
    except subprocess.CalledProcessError as e:
        typer.echo(f"❌ Failed to install core: {e}", err=True)


@arduino_command.command()
def multiplexer(device: str = "/dev/ttyACM0"):
    """Start the Arduino multiplexer."""
    arduino = Arduino(device)
    if arduino.start_multiplexer():
        typer.echo("Arduino multiplexer started successfully!")
    else:
        typer.echo("Failed to start Arduino multiplexer.")


@arduino_command.command()
def debug():
    """Start the Arduino debug terminal (send-only)."""
    Arduino.arduino_debug_terminal()


@arduino_command.command()
def monitor():
    """Start the Arduino monitor (real-time monitoring)."""
    Arduino.arduino_monitor()


@arduino_command.command()
def virtual(device: str = "/dev/ttyACM0"):
    """Start a virtual Arduino device if no real Arduino is present."""
    va = check_and_create_virtual_arduino(device)
    if va:
        typer.echo(f"Virtual Arduino started at {device}. Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            va.stop()
    else:
        typer.echo("Using real Arduino device.")
