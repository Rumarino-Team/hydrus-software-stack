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


class HydrusCLI:
    """Central CLI for all Hydrus software operations - the intermediate layer"""

    def __init__(self):
        self.script_dir = Path(__file__).parent.absolute()
        self.hydrus_root = self._find_hydrus_root()
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
Primary Commands:
  build                Build the ROS workspace
  test                 Run the complete test suite
  tmux                 Start tmux monitoring sessions
  monitor              Start monitoring and debugging tools

Hardware Control:
  arduino-compile      Compile and upload Arduino code
  virtual-arduino      Start virtual Arduino processes
  serial-bridge        Start serial ROS bridge

Data & Simulation:
  rosbag-download      Download rosbag files
  rosbag-play          Start rosbag playback
  rosbag-convert       Convert video to rosbag

Visualization & UI:
  rviz                 Start RViz visualization
  web-ui               Start web user interface
  api-server           Start API server

Development Tools:
  format               Format code with project standards
  lint                 Run code quality checks
  profile              Start ROS node profiler

Examples:
  # Complete development setup
  hydrus-cli build --tmux --arduino-compile --virtual-arduino

  # Testing workflow
  hydrus-cli test --rosbag-download

  # Simulation setup
  hydrus-cli rosbag-download --rosbag-play --rviz --web-ui

  # Production deployment
  hydrus-cli build --arduino-compile --tmux --monitor

  # Quick development monitoring
  hydrus-cli tmux --profile
            """,
        )

        # Primary actions
        parser.add_argument(
            "action",
            nargs="?",
            choices=[
                "build",
                "test",
                "tmux",
                "monitor",
                "arduino-compile",
                "virtual-arduino",
                "serial-bridge",
                "rosbag-download",
                "rosbag-play",
                "rosbag-convert",
                "rviz",
                "web-ui",
                "api-server",
                "format",
                "lint",
                "profile",
            ],
            help="Primary action to perform",
        )

        # Build options
        parser.add_argument(
            "--no-build", action="store_true", help="Skip catkin workspace build"
        )
        parser.add_argument(
            "--clean", action="store_true", help="Clean workspace before building"
        )

        # Test options
        parser.add_argument(
            "--test", action="store_true", help="Run tests after other actions"
        )
        parser.add_argument(
            "--test-integration", action="store_true", help="Run integration tests only"
        )
        parser.add_argument(
            "--test-unit", action="store_true", help="Run unit tests only"
        )

        # Monitoring & Development
        parser.add_argument(
            "--tmux", action="store_true", help="Start tmux monitoring sessions"
        )
        parser.add_argument(
            "--monitor",
            action="store_true",
            help="Start monitoring and debugging tools",
        )
        parser.add_argument(
            "--profile", action="store_true", help="Start ROS node profiler"
        )

        # Hardware options
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

        # Data & Simulation options
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
            "--video-file", type=str, help="Video file to convert to rosbag"
        )

        # Visualization & UI
        parser.add_argument(
            "--rviz", action="store_true", help="Start RViz visualization"
        )
        parser.add_argument(
            "--web-ui", action="store_true", help="Start web user interface"
        )
        parser.add_argument(
            "--api-server", action="store_true", help="Start API server"
        )

        # Development tools
        parser.add_argument(
            "--format", action="store_true", help="Format code with project standards"
        )
        parser.add_argument(
            "--lint", action="store_true", help="Run code quality checks"
        )

        # Configuration
        parser.add_argument(
            "--config",
            choices=["test", "development", "simulation", "production", "competition"],
            help="Use predefined configuration",
        )

        # Ports and devices
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

        return parser

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

        test_script = self.hydrus_root / "scripts/run_tests.py"
        if test_script.exists():
            result = self.run_command([sys.executable, str(test_script)], check=False)
            return result.returncode == 0
        else:
            print("❌ Test script not found")
            return False

    def start_tmux_sessions(self):
        """Start tmux monitoring sessions"""
        print("📺 Starting tmux monitoring sessions...")

        tmux_script = self.hydrus_root / "scripts/start_tmux_sessions.py"
        if tmux_script.exists():
            self.run_command([sys.executable, str(tmux_script)], check=False)
        else:
            # Fallback to bash script
            tmux_script = self.hydrus_root / "start_tmux_sessions.sh"
            if tmux_script.exists():
                self.run_command(["bash", str(tmux_script)], check=False)
            else:
                print("❌ Tmux script not found")

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

        virtual_script = self.hydrus_root / "scripts/virtual_arduino.py"
        if not virtual_script.exists():
            print("❌ Virtual Arduino script not found")
            return False

        for port in ["/dev/ttyACM0", "/dev/ttyACM1"]:
            try:
                subprocess.Popen(
                    [sys.executable, str(virtual_script), port],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                print(f"✅ Started virtual Arduino for {port}")
                time.sleep(1)
            except Exception as e:
                print(f"❌ Failed to start virtual Arduino for {port}: {e}")

        return True

    def download_rosbags(self):
        """Download rosbag files"""
        print("📦 Downloading rosbag files...")

        download_script = self.hydrus_root / "scripts/download_rosbag.py"
        if download_script.exists():
            result = self.run_command(
                [sys.executable, str(download_script)], check=False
            )
            return result.returncode == 0
        else:
            print("❌ Rosbag download script not found")
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

        # Start detection viewer
        detection_viewer = self.hydrus_root / "autonomy/scripts/web/detection_viewer.py"
        if detection_viewer.exists():
            try:
                subprocess.Popen(
                    [sys.executable, str(detection_viewer)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                print("✅ Web detection viewer started")
            except Exception as e:
                print(f"❌ Failed to start detection viewer: {e}")
                return False

        return True

    def start_api_server(self):
        """Start API server"""
        print("🖥️  Starting API server...")

        api_server = self.hydrus_root / "autonomy/src/api_server.py"
        if not api_server.exists():
            # Try alternative location
            api_server = self.hydrus_root / "autonomy/src/API.py"

        if api_server.exists():
            try:
                subprocess.Popen(
                    [sys.executable, str(api_server)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                print("✅ API server started")
                return True
            except Exception as e:
                print(f"❌ Failed to start API server: {e}")
                return False
        else:
            print("❌ API server script not found")
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

        converter_script = self.hydrus_root / "scripts/video_to_rosbag.py"
        if not converter_script.exists():
            print("❌ Video converter script not found")
            return False

        if not output_bag:
            output_bag = Path(video_file).stem + ".bag"

        try:
            result = self.run_command(
                [sys.executable, str(converter_script), video_file, output_bag],
                check=False,
            )
            return result.returncode == 0
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

    def main(self):
        """Main execution function"""
        try:
            parser = self.create_parser()
            args = parser.parse_args()

            # Determine workspace
            workspace_dir, is_volume = self.determine_workspace_type()
            print(
                f"📁 Using workspace: {workspace_dir} ({'volume' if is_volume else 'container'})"
            )

            success = True

            # Apply configuration if specified
            if args.config:
                config = self.apply_config(args.config)
                if config:
                    print(f"🔧 Applying {args.config} configuration")
                    # Override args with config
                    for action in config["actions"]:
                        setattr(args, action.replace("-", "_"), True)
                    for option, value in config["options"].items():
                        setattr(args, option, value)

            # Execute primary action
            if args.action == "build" or args.action is None:
                if args.clean:
                    print("🧹 Cleaning workspace...")
                    clean_cmd = f"cd {workspace_dir} && catkin_make clean"
                    subprocess.run(["bash", "-c", clean_cmd], check=False)
                if not args.no_build:
                    success &= self.build_workspace(workspace_dir)

            elif args.action == "test":
                success &= self.run_tests()

            elif args.action == "arduino-compile":
                success &= self.compile_arduino()
            elif args.action == "tmux":
                self.start_tmux_sessions()

            elif args.action == "monitor":
                success &= self.start_monitoring()

            elif args.action == "virtual-arduino":
                success &= self.start_virtual_arduino()

            elif args.action == "serial-bridge":
                success &= self.start_serial_bridge(
                    args.port, getattr(args, "baud_rate", 115200)
                )

            elif args.action == "rosbag-download":
                success &= self.download_rosbags()

            elif args.action == "rosbag-play":
                success &= self.start_rosbag_playback(
                    getattr(args, "rosbag_loop", False)
                )

            elif args.action == "rosbag-convert":
                if getattr(args, "video_file", None):
                    success &= self.convert_video_to_rosbag(args.video_file)
                else:
                    print("❌ --video-file required for rosbag-convert")
                    success = False

            elif args.action == "rviz":
                success &= self.start_rviz()

            elif args.action == "web-ui":
                success &= self.start_web_ui()

            elif args.action == "api-server":
                success &= self.start_api_server()

            elif args.action == "format":
                success &= self.format_code()

            elif args.action == "lint":
                success &= self.lint_code()

            elif args.action == "profile":
                success &= self.start_profiler()

            # Execute additional options (flags that can be combined)
            if args.test and args.action != "test":
                if args.test_integration:
                    print("🧪 Running integration tests...")
                    # Add integration test logic here
                elif args.test_unit:
                    print("🧪 Running unit tests...")
                    # Add unit test logic here
                else:
                    success &= self.run_tests()

            if args.tmux and args.action != "tmux":
                self.start_tmux_sessions()

            if args.monitor and args.action != "monitor":
                success &= self.start_monitoring()

            if (
                getattr(args, "arduino_compile", False)
                and args.action != "arduino-compile"
            ):
                success &= self.compile_arduino()

            if (
                getattr(args, "virtual_arduino", False)
                and args.action != "virtual-arduino"
            ):
                success &= self.start_virtual_arduino()

            if getattr(args, "serial_bridge", False) and args.action != "serial-bridge":
                success &= self.start_serial_bridge(
                    args.port, getattr(args, "baud_rate", 115200)
                )

            if (
                getattr(args, "rosbag_download", False)
                and args.action != "rosbag-download"
            ):
                success &= self.download_rosbags()

            if getattr(args, "rosbag_play", False) and args.action != "rosbag-play":
                success &= self.start_rosbag_playback(
                    getattr(args, "rosbag_loop", False)
                )

            if args.rviz and args.action != "rviz":
                success &= self.start_rviz()

            if getattr(args, "web_ui", False) and args.action != "web-ui":
                success &= self.start_web_ui()

            if getattr(args, "api_server", False) and args.action != "api-server":
                success &= self.start_api_server()

            if args.format and args.action != "format":
                success &= self.format_code()

            if args.lint and args.action != "lint":
                success &= self.lint_code()

            if args.profile and args.action != "profile":
                success &= self.start_profiler()

            if success:
                print("\n✅ All operations completed successfully")
                print("\n💡 Hydrus software is ready!")
                print("📖 Use 'hydrus-cli --help' for more options")
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
