"""
Hydrus Software Control CLI
Complete intermediate layer for all Hydrus software operations
Separated from Docker management - handles all Hydrus software control
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

# Add project directories to path for direct imports
project_root = Path(__file__).parent.parent.absolute()
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "scripts"))

# Import script modules directly (conditionally to avoid import errors)
try:
    from scripts import (
        download_rosbag,
        run_tests,
        start_tmux_sessions,
        video_to_rosbag,
        virtual_arduino,
    )

    SCRIPTS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import some scripts: {e}")
    SCRIPTS_AVAILABLE = False

try:
    from autonomy.scripts.web import detection_viewer
    from autonomy.src import api_server

    AUTONOMY_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import autonomy modules: {e}")
    AUTONOMY_AVAILABLE = False


class HydrusCLI:
    """Central CLI for all Hydrus software operations - the intermediate layer"""

    def __init__(self):
        self.script_dir = Path(__file__).parent.absolute()
        self.hydrus_root = self._find_hydrus_root()
        # Add project directories to path for direct imports
        sys.path.insert(0, str(self.hydrus_root))
        print("🔧 HYDRUS SOFTWARE CONTROL CLI")
        print("=" * 50)

    def _find_hydrus_root(self):
        """Find the hydrus-software-stack root directory"""
        # Look for the directory based on common container paths
        possible_paths = [
            Path("/catkin_ws/src/hydrus-software-stack"),
            Path("/home/catkin_ws/src/hydrus-software-stack"),
            Path.cwd() / "hydrus-software-stack",
            Path.cwd(),
        ]

        for path in possible_paths:
            if path.exists() and (path / "autonomy").exists():
                return path

        raise RuntimeError("Could not find hydrus-software-stack directory")

    def create_parser(self):
        """Create argument parser for hydrus-cli"""
        parser = argparse.ArgumentParser(
            description="Hydrus Software Control CLI - Complete interface for all Hydrus operations",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog="""
Available Commands:
  build                Build the ROS workspace
  tmux                 Start and manage tmux monitoring sessions
  test                 Run the complete test suite

Global Options (can be used with any command):
  --arduino-compile     Compile and upload Arduino code
  --virtual-arduino     Start virtual Arduino processes
  --serial-bridge       Start serial ROS bridge
  --rosbag-download     Download rosbag files
  --rosbag-play         Start rosbag playback
  --rviz                Start RViz visualization
  --web-ui              Start web user interface
  --api-server          Start API server
  --monitor             Start monitoring and debugging tools
  --format              Format code with project standards

Examples:
  # Build workspace with Arduino compilation
  hydrus-cli build --arduino-compile

  # Start tmux with specific windows
  hydrus-cli tmux --windows Controls Arduino

  # Start tmux with global options
  hydrus-cli tmux --arduino-compile --virtual-arduino

  # Get tmux-specific help
  hydrus-cli tmux --help

  # Run tests with rosbag download
  hydrus-cli test --rosbag-download
            """,
        )

        # Global options that can be used with any command
        parser.add_argument(
            "--arduino-compile",
            action="store_true",
            help="Compile and upload Arduino code",
        )
        parser.add_argument(
            "--virtual-arduino",
            action="store_true",
            help="Start virtual Arduino processes",
        )
        parser.add_argument(
            "--serial-bridge", action="store_true", help="Start serial ROS bridge"
        )
        parser.add_argument(
            "--rosbag-download",
            action="store_true",
            help="Download rosbag files if missing",
        )
        parser.add_argument(
            "--rosbag-play", action="store_true", help="Start rosbag playback"
        )
        parser.add_argument(
            "--rosbag-loop", action="store_true", help="Loop rosbag playback"
        )
        parser.add_argument(
            "--rviz", action="store_true", help="Start RViz visualization"
        )
        parser.add_argument(
            "--web-ui", action="store_true", help="Start web user interface"
        )
        parser.add_argument(
            "--api-server", action="store_true", help="Start API server"
        )
        parser.add_argument(
            "--monitor",
            action="store_true",
            help="Start monitoring and debugging tools",
        )
        parser.add_argument(
            "--format", action="store_true", help="Format code with project standards"
        )
        parser.add_argument(
            "--port",
            default="/dev/ttyACM0",
            help="Serial port for Arduino (default: /dev/ttyACM0)",
        )
        parser.add_argument(
            "--baud-rate",
            type=int,
            default=115200,
            help="Baud rate for serial communication",
        )

        # Create subparsers for commands
        subparsers = parser.add_subparsers(
            dest="command", help="Available commands", metavar="<command>"
        )

        # Build subcommand
        build_parser = subparsers.add_parser(
            "build",
            help="Build the ROS workspace",
            description="Build the catkin workspace",
        )
        build_parser.add_argument(
            "--clean", action="store_true", help="Clean workspace before building"
        )
        build_parser.add_argument(
            "--no-build", action="store_true", help="Skip catkin workspace build"
        )
        build_parser.set_defaults(func=self._handle_build_command)

        # Test subcommand
        test_parser = subparsers.add_parser(
            "test",
            help="Run the complete test suite",
            description="Run Hydrus test suite",
        )
        test_parser.add_argument(
            "--integration", action="store_true", help="Run integration tests only"
        )
        test_parser.add_argument(
            "--unit", action="store_true", help="Run unit tests only"
        )
        test_parser.set_defaults(func=self._handle_test_command)

        # Register tmux subcommand from start_tmux_sessions
        if SCRIPTS_AVAILABLE:
            try:
                start_tmux_sessions.register_subcommand(subparsers)
            except AttributeError:
                print("Warning: Could not register tmux subcommand")

        return parser

    def _handle_build_command(self, args):
        """Handle build subcommand"""
        workspace_dir, _ = self.determine_workspace_type()

        if args.clean:
            print("🧹 Cleaning workspace...")
            clean_cmd = f"cd {workspace_dir} && catkin_make clean"
            subprocess.run(["bash", "-c", clean_cmd], check=False)

        if not args.no_build:
            success = self.build_workspace(workspace_dir)
            return success
        return True

    def _handle_test_command(self, args):
        """Handle test subcommand"""
        success = self.run_tests()
        return success

    def _execute_global_options(self, args):
        """Execute global options that can be combined with any command"""
        success = True

        if args.arduino_compile:
            success &= self.compile_arduino()

        if args.virtual_arduino:
            success &= self.start_virtual_arduino()

        if args.serial_bridge:
            success &= self.start_serial_bridge(args.port, args.baud_rate)

        if args.rosbag_download:
            success &= self.download_rosbags()

        if args.rosbag_play:
            success &= self.start_rosbag_playback(args.rosbag_loop)

        if args.rviz:
            success &= self.start_rviz()

        if args.web_ui:
            success &= self.start_web_ui()

        if args.api_server:
            success &= self.start_api_server()

        if args.monitor:
            success &= self.start_monitoring()

        if args.format:
            success &= self.format_code()

        return success

    def determine_workspace_type(self):
        """Determine if we're in volume mount or container copy mode"""
        volume_path = Path("/home/catkin_ws")
        container_path = Path("/catkin_ws")

        if volume_path.exists() and (volume_path / "src").exists():
            return volume_path, True
        elif container_path.exists() and (container_path / "src").exists():
            return container_path, False
        else:
            raise RuntimeError("Could not find catkin workspace")

    def run_command(self, cmd, check=True, capture_output=False, env=None, cwd=None):
        """Run a command with proper error handling"""
        if isinstance(cmd, str):
            cmd = cmd.split()

        print(f"🚀 Running: {' '.join(cmd)}")

        try:
            result = subprocess.run(
                cmd,
                check=check,
                capture_output=capture_output,
                text=True,
                env=env,
                cwd=cwd,
            )
            return result
        except subprocess.CalledProcessError as e:
            print(f"❌ Command failed with exit code {e.returncode}")
            if e.stdout:
                print(f"STDOUT: {e.stdout}")
            if e.stderr:
                print(f"STDERR: {e.stderr}")
            if check:
                sys.exit(e.returncode)
            return e

    def setup_ros_environment(self, workspace_dir):
        """Setup ROS environment"""
        env = os.environ.copy()

        # Add workspace to environment
        if (workspace_dir / "devel/setup.bash").exists():
            env[
                "ROS_PACKAGE_PATH"
            ] = f"{workspace_dir}/src:{env.get('ROS_PACKAGE_PATH', '')}"
            env[
                "CMAKE_PREFIX_PATH"
            ] = f"{workspace_dir}/devel:{env.get('CMAKE_PREFIX_PATH', '')}"
            env[
                "LD_LIBRARY_PATH"
            ] = f"{workspace_dir}/devel/lib:{env.get('LD_LIBRARY_PATH', '')}"
            env[
                "PYTHONPATH"
            ] = f"{workspace_dir}/devel/lib/python3/dist-packages:{env.get('PYTHONPATH', '')}"

        return env

    def build_workspace(self, workspace_dir):
        """Build the catkin workspace"""
        print("🔨 Building catkin workspace...")

        env = self.setup_ros_environment(workspace_dir)

        # Source ROS setup first
        source_cmd = f"source /opt/ros/{env.get('ROS_DISTRO', 'noetic')}/setup.bash"
        build_cmd = f"{source_cmd} && cd {workspace_dir} && catkin_make"

        result = subprocess.run(["bash", "-c", build_cmd], env=env, check=False)

        if result.returncode == 0:
            print("✅ Workspace build completed successfully")
            return True
        else:
            print("❌ Workspace build failed")
            return False

    def run_tests(self):
        """Run the test suite"""
        print("🧪 Running Hydrus test suite...")
        try:
            # Assuming run_tests.py has a main() function
            run_tests.main()
            return True
        except Exception as e:
            print(f"❌ Test suite failed: {e}")
            return False

    def start_tmux_sessions(self):
        """Start tmux monitoring sessions"""
        print("📺 Starting tmux monitoring sessions...")
        try:
            # Assuming start_tmux_sessions.py has a main() function
            start_tmux_sessions.main()
        except Exception as e:
            print(f"❌ Failed to start tmux sessions: {e}")

    def compile_arduino(self):
        """Compile and upload Arduino code"""
        print("🔧 Compiling and uploading Arduino code...")

        arduino_dir = Path("/root/Arduino/libraries/embedded_arduino/Hydrus")
        if not arduino_dir.exists():
            print("❌ Arduino directory not found")
            return False

        arduino_board = os.environ.get("ARDUINO_BOARD", "arduino:avr:mega")

        # Compile
        compile_result = self.run_command(
            ["arduino-cli", "compile", "--fqbn", arduino_board, "Hydrus.ino"],
            check=False,
            cwd=arduino_dir,
        )

        if compile_result.returncode != 0:
            print("❌ Arduino compilation failed")
            return False

        # Upload with retry logic
        ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]
        for port in ports:
            if Path(port).exists():
                print(f"📤 Uploading to {port}...")
                upload_result = self.run_command(
                    [
                        "arduino-cli",
                        "upload",
                        "-p",
                        port,
                        "--fqbn",
                        arduino_board,
                        "Hydrus.ino",
                    ],
                    check=False,
                    cwd=arduino_dir,
                )

                if upload_result.returncode == 0:
                    print(f"✅ Upload successful to {port}")
                    return True

        print("❌ Arduino upload failed on all ports")
        return False

    def start_virtual_arduino(self):
        """Start virtual Arduino processes"""
        print("🤖 Starting virtual Arduino processes...")
        try:
            # Assuming virtual_arduino.py has a main() function
            virtual_arduino.main()
            return True
        except Exception as e:
            print(f"❌ Failed to start virtual Arduino: {e}")
            return False

    def download_rosbags(self):
        """Download rosbag files"""
        print("📦 Downloading rosbag files...")
        try:
            # Assuming download_rosbag.py has a main() function
            download_rosbag.main()
            return True
        except Exception as e:
            print(f"❌ Failed to download rosbags: {e}")
            return False

    def start_rosbag_playback(self, loop=False):
        """Start rosbag playback"""
        print("🎬 Starting rosbag playback...")

        rosbags_dir = Path("/rosbags")
        if not rosbags_dir.exists():
            print("❌ /rosbags directory not found")
            return False

        bag_files = list(rosbags_dir.glob("*.bag"))
        if not bag_files:
            print("❌ No rosbag files found")
            return False

        # Get most recent bag file
        most_recent = max(bag_files, key=lambda x: x.stat().st_mtime)
        print(f"📂 Playing: {most_recent}")

        cmd = ["rosbag", "play", str(most_recent)]
        if loop:
            cmd.append("--loop")

        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            loop_text = " (looping)" if loop else ""
            print(f"✅ Rosbag playback started{loop_text}")
            return True
        except Exception as e:
            print(f"❌ Failed to start rosbag playback: {e}")
            return False

    def start_rviz(self):
        """Start RViz visualization"""
        print("👁️  Starting RViz...")

        config_file = self.hydrus_root / "autonomy/config/cv_detection.rviz"

        try:
            if config_file.exists():
                subprocess.Popen(
                    ["rviz", "-d", str(config_file)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            else:
                subprocess.Popen(
                    ["rviz"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )

            print("✅ RViz started")
            return True
        except Exception as e:
            print(f"❌ Failed to start RViz: {e}")
            return False

    def apply_config(self, config_name):
        """Apply predefined configuration"""
        configs = {
            "test": {
                "actions": ["build", "test"],
                "options": {"rosbag_download": True},
            },
            "development": {
                "actions": ["build", "tmux", "monitor"],
                "options": {
                    "arduino_compile": True,
                    "virtual_arduino": True,
                    "rosbag_download": True,
                    "serial_bridge": True,
                },
            },
            "simulation": {
                "actions": [
                    "build",
                    "rosbag-download",
                    "rosbag-play",
                    "rviz",
                    "web-ui",
                ],
                "options": {"rosbag_loop": True},
            },
            "production": {
                "actions": ["build", "tmux", "arduino-compile", "monitor"],
                "options": {"serial_bridge": True},
            },
            "competition": {
                "actions": ["build", "arduino-compile", "tmux"],
                "options": {"serial_bridge": True, "monitor": True},
            },
        }

        if config_name in configs:
            return configs[config_name]
        return None

    def start_serial_bridge(self, port="/dev/ttyACM0", baud_rate=115200):
        """Start serial ROS bridge"""
        print("🔗 Starting serial ROS bridge...")

        bridge_script = (
            self.hydrus_root / "autonomy/scripts/controller/serial_ros_bridge.py"
        )
        if not bridge_script.exists():
            print("❌ Serial ROS bridge script not found")
            return False

        try:
            subprocess.Popen(
                [
                    sys.executable,
                    str(bridge_script),
                    f"_port:={port}",
                    f"_baud_rate:={baud_rate}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print(f"✅ Serial ROS bridge started on {port}")
            return True
        except Exception as e:
            print(f"❌ Failed to start serial ROS bridge: {e}")
            return False

    def start_web_ui(self):
        """Start web user interface"""
        print("🌐 Starting web user interface...")
        try:
            # Run in a separate process so it doesn't block
            subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "from autonomy.scripts.web import detection_viewer; detection_viewer.main()",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print("✅ Web detection viewer started")
            return True
        except Exception as e:
            print(f"❌ Failed to start detection viewer: {e}")
            return False

    def start_api_server(self):
        """Start API server"""
        print("🖥️  Starting API server...")
        try:
            # Run in a separate process so it doesn't block
            subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "from autonomy.src import api_server; api_server.main()",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print("✅ API server started")
            return True
        except Exception as e:
            print(f"❌ Failed to start API server: {e}")
            return False

    def start_monitoring(self):
        """Start monitoring and debugging tools"""
        print("📊 Starting monitoring tools...")

        success = True

        # Start ROS profiler
        profiler_script = self.hydrus_root / "autonomy/scripts/services/ros_profiler.py"
        if profiler_script.exists():
            try:
                subprocess.Popen(
                    [sys.executable, str(profiler_script)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                print("✅ ROS profiler started")
            except Exception as e:
                print(f"⚠️  ROS profiler failed: {e}")
                success = False

        return success

    def convert_video_to_rosbag(self, video_file, output_bag=None):
        """Convert video file to rosbag"""
        print(f"🎬 Converting video to rosbag: {video_file}")

        if not Path(video_file).exists():
            print(f"❌ Video file not found: {video_file}")
            return False

        if not output_bag:
            output_bag = Path(video_file).stem + ".bag"

        try:
            # Assuming video_to_rosbag.py has a main() function that takes arguments
            video_to_rosbag.main([video_file, output_bag])
            return True
        except Exception as e:
            print(f"❌ Failed to convert video: {e}")
            return False

    def format_code(self):
        """Format code with project standards"""
        print("🎨 Formatting code...")

        format_script = self.hydrus_root / "format.sh"
        if format_script.exists():
            result = self.run_command(["bash", str(format_script)], check=False)
            return result.returncode == 0
        else:
            print("❌ Format script not found")
            return False

    def lint_code(self):
        """Run code quality checks"""
        print("🔍 Running code quality checks...")

        # This would run linting tools
        print("⚠️  Linting functionality not yet implemented")
        return True

    def start_profiler(self):
        """Start ROS node profiler"""
        print("📈 Starting ROS node profiler...")

        profiler_script = self.hydrus_root / "autonomy/scripts/services/ros_profiler.py"
        if profiler_script.exists():
            try:
                subprocess.Popen(
                    [sys.executable, str(profiler_script)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                print("✅ ROS profiler started")
                return True
            except Exception as e:
                print(f"❌ Failed to start profiler: {e}")
                return False
        else:
            print("❌ Profiler script not found")
            return False

    def get_action_priorities(self):
        """Define priority for each action (0 = highest priority)"""
        return {
            # Critical hardware setup (highest priority)
            "arduino_compile": 0,
            "build": 1,
            # Core services
            "virtual_arduino": 2,
            "serial_bridge": 3,
            # Data preparation
            "rosbag_download": 4,
            "rosbag_convert": 5,
            # Monitoring and visualization (should start after core services)
            "tmux": 6,
            "monitor": 7,
            "profile": 8,
            # Playback and simulation
            "rosbag_play": 9,
            "rviz": 10,
            # Web services
            "api_server": 11,
            "web_ui": 12,
            # Testing and development tools
            "test": 13,
            "format": 14,
            "lint": 15,
        }

    def collect_requested_actions(self, args):
        """Collect all requested actions from arguments"""
        requested_actions = []

        # Check primary action
        if args.action:
            action_name = args.action.replace("-", "_")
            requested_actions.append(action_name)

        # Check all flag-based actions
        flag_actions = [
            "build",
            "test",
            "tmux",
            "monitor",
            "profile",
            "arduino_compile",
            "virtual_arduino",
            "serial_bridge",
            "rosbag_download",
            "rosbag_play",
            "rosbag_convert",
            "rviz",
            "web_ui",
            "api_server",
            "format",
            "lint",
        ]

        for action in flag_actions:
            if hasattr(args, action) and getattr(args, action):
                if action not in requested_actions:  # Avoid duplicates
                    requested_actions.append(action)

        return requested_actions

    def execute_actions_by_priority(self, args, workspace_dir):
        """Execute all requested actions in priority order"""
        priorities = self.get_action_priorities()
        requested_actions = self.collect_requested_actions(args)

        if not requested_actions:
            print("ℹ️  No actions requested, defaulting to build")
            requested_actions = ["build"]

        # Sort actions by priority (0 = highest priority)
        sorted_actions = sorted(
            requested_actions,
            key=lambda x: priorities.get(
                x, 999
            ),  # 999 = lowest priority for unknown actions
        )

        print("🔄 PRIORITY-BASED EXECUTION ORDER")
        print("=" * 50)
        for i, action in enumerate(sorted_actions, 1):
            priority = priorities.get(action, 999)
            print(f"   {i}. {action.replace('_', '-')} (priority: {priority})")
        print("=" * 50)

        success = True

        for action in sorted_actions:
            print(f"\n🚀 Executing: {action.replace('_', '-')}")

            if action == "build":
                if getattr(args, "clean", False):
                    print("🧹 Cleaning workspace...")
                    clean_cmd = f"cd {workspace_dir} && catkin_make clean"
                    subprocess.run(["bash", "-c", clean_cmd], check=False)
                if not getattr(args, "no_build", False):
                    success &= self.build_workspace(workspace_dir)

            elif action == "arduino_compile":
                success &= self.compile_arduino()
                # Add delay after Arduino compile to ensure upload is complete
                print("⏳ Waiting for Arduino upload to stabilize...")
                time.sleep(5)

            elif action == "test":
                success &= self.run_tests()

            elif action == "tmux":
                self.start_tmux_sessions()

            elif action == "monitor":
                success &= self.start_monitoring()

            elif action == "profile":
                success &= self.start_profiler()

            elif action == "virtual_arduino":
                success &= self.start_virtual_arduino()

            elif action == "serial_bridge":
                success &= self.start_serial_bridge(
                    getattr(args, "port", "/dev/ttyACM0"),
                    getattr(args, "baud_rate", 115200),
                )

            elif action == "rosbag_download":
                success &= self.download_rosbags()

            elif action == "rosbag_play":
                success &= self.start_rosbag_playback(
                    getattr(args, "rosbag_loop", False)
                )

            elif action == "rosbag_convert":
                if getattr(args, "video_file", None):
                    success &= self.convert_video_to_rosbag(args.video_file)
                else:
                    print("❌ --video-file required for rosbag-convert")
                    success = False

            elif action == "rviz":
                success &= self.start_rviz()

            elif action == "web_ui":
                success &= self.start_web_ui()

            elif action == "api_server":
                success &= self.start_api_server()

            elif action == "format":
                success &= self.format_code()

            elif action == "lint":
                success &= self.lint_code()

            else:
                print(f"⚠️  Unknown action: {action}")

        return success

    def main(self):
        """Main execution function with subparser-based architecture"""
        try:
            parser = self.create_parser()
            args = parser.parse_args()

            # If no command specified, show help
            if not hasattr(args, "func"):
                parser.print_help()
                sys.exit(0)

            # Execute global options first
            global_success = self._execute_global_options(args)

            # Execute the specific subcommand
            command_success = args.func(args)

            # Both global options and command must succeed
            success = global_success and command_success

            if success:
                print("\n✅ All operations completed successfully")
                sys.exit(0)
            else:
                print("\n❌ Some operations failed")
                sys.exit(1)

        except KeyboardInterrupt:
            print("\n🛑 Operation cancelled by user")
            sys.exit(1)
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
            sys.exit(1)


if __name__ == "__main__":
    cli = HydrusCLI()
    cli.main()
