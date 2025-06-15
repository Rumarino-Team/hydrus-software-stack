#!/usr/bin/env python3
"""
Hydrus ROS Entrypoint Script
Replaces ros-entrypoint.sh with improved modularity and error handling
"""

import os
import platform
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional


class HydrusROSManager:
    def __init__(self):
        self.script_dir = Path(__file__).parent.absolute()

        # Print environment variables for debugging
        print("=" * 60)
        print("🔧 HYDRUS ROS ENVIRONMENT CONFIGURATION")
        print("=" * 60)

        # Get all environment variables we use
        env_vars = {
            "RVIZ_CONFIG": os.environ.get("RVIZ_CONFIG", "cv_detection.rviz"),
            "VOLUME": os.environ.get("VOLUME", "false"),
            "DEPLOY": os.environ.get("DEPLOY", "false"),
            "TEST": os.environ.get("TEST", "false"),
            "ROSBAG_PLAYBACK": os.environ.get("ROSBAG_PLAYBACK", "false"),
            "RVIZ": os.environ.get("RVIZ", "false"),
            "ZED_OPTION": os.environ.get("ZED_OPTION", "false"),
            "DEBUG_ARDUINO": os.environ.get("DEBUG_ARDUINO", "false"),
            "ARDUINO_BOARD": os.environ.get("ARDUINO_BOARD", "arduino:avr:mega"),
            "ROS_DISTRO": os.environ.get("ROS_DISTRO", "not set"),
            "ROS_PACKAGE_PATH": os.environ.get("ROS_PACKAGE_PATH", "not set"),
            "NO_BUILD": os.environ.get("NO_BUILD", "false"),
        }

        for var_name, var_value in env_vars.items():
            print(f"📋 {var_name:<20} = {var_value}")

        print("=" * 60)
        print()

        # Parse environment variables
        self.rviz_config = env_vars["RVIZ_CONFIG"]
        self.volume = env_vars["VOLUME"].lower() == "true"
        self.deploy = env_vars["DEPLOY"].lower() == "true"
        self.test = env_vars["TEST"].lower() == "true"
        self.rosbag_playback = env_vars["ROSBAG_PLAYBACK"].lower() == "true"
        self.rviz = env_vars["RVIZ"].lower() == "true"
        self.no_build = False  # env_vars["NO_BUILD"].lower() == "true"

        # Determine ROS directory based on volume usage
        if self.volume:
            print("Using the Volume directory for building the packages.")
            self.ros_dir = Path("/home/catkin_ws")
        else:
            print("Using the Copied Packages from Docker.")
            self.ros_dir = Path("/catkin_ws")

        print(f"Using ROS workspace: {self.ros_dir}")

        # Detect ROS distribution
        self.ros_distro = self._detect_ros_distro()

        # Background process tracking
        self.background_processes = []

    def _detect_ros_distro(self) -> str:
        """Detect ROS distribution (Noetic or Melodic)"""
        if Path("/opt/ros/noetic").exists():
            print("Detected ROS Noetic")
            return "noetic"
        elif Path("/opt/ros/melodic").exists():
            print("Detected ROS Melodic")
            return "melodic"
        else:
            print("No compatible ROS distribution found!")
            sys.exit(1)

    def _run_command(
        self,
        cmd: List[str],
        check: bool = True,
        capture_output: bool = False,
        env: Optional[Dict] = None,
        cwd: Optional[Path] = None,
    ) -> subprocess.CompletedProcess:
        """Run a command with proper error handling"""
        if env is None:
            env = os.environ.copy()

        try:
            return subprocess.run(
                cmd,
                check=check,
                capture_output=capture_output,
                env=env,
                cwd=cwd,
                text=True,
            )
        except subprocess.CalledProcessError as e:
            if check:
                print(f"Command failed: {' '.join(cmd)}")
                print(f"Exit code: {e.returncode}")
                if capture_output and e.stderr:
                    print(f"Error: {e.stderr}")
            raise

    def _setup_ros_environment(self):
        """Setup ROS environment by sourcing ROS setup.bash"""
        print(f"Setting up ROS {self.ros_distro} environment...")

        # Get the ROS environment by sourcing setup.bash
        ros_setup = f"/opt/ros/{self.ros_distro}/setup.bash"

        # Use bash to source the ROS setup and print the environment
        cmd = f"source {ros_setup} && env"

        try:
            result = subprocess.run(
                ["bash", "-c", cmd],
                capture_output=True,
                text=True,
                check=True,
            )

            # Parse the environment variables from the output
            env = os.environ.copy()
            for line in result.stdout.split("\n"):
                if "=" in line and not line.startswith("_"):
                    key, value = line.split("=", 1)
                    env[key] = value

            # Ensure ROS_DISTRO is set
            env["ROS_DISTRO"] = self.ros_distro

            print(f"✅ ROS environment configured for {self.ros_distro}")
            print(f"   ROS_PACKAGE_PATH: {env.get('ROS_PACKAGE_PATH', 'not set')}")
            print(f"   CMAKE_PREFIX_PATH: {env.get('CMAKE_PREFIX_PATH', 'not set')}")

            return env

        except subprocess.CalledProcessError as e:
            print(f"❌ Failed to source ROS setup: {e}")
            print(f"   stderr: {e.stderr}")
            # Fallback to basic environment
            env = os.environ.copy()
            env["ROS_DISTRO"] = self.ros_distro
            if "ROS_PACKAGE_PATH" not in env:
                env["ROS_PACKAGE_PATH"] = f"/opt/ros/{self.ros_distro}/share"
            return env

    def _build_workspace(self):
        """Build the catkin workspace"""
        print("Building catkin workspace...")
        env = self._setup_ros_environment()

        # Use bash to run catkin_make with proper ROS environment
        ros_setup = f"/opt/ros/{self.ros_distro}/setup.bash"

        # Build command that sources ROS and runs catkin_make
        build_cmd = f"""
        source {ros_setup} &&
        cd {self.ros_dir} &&
        catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3
        """

        try:
            print("🔨 Running catkin_make...")
            result = subprocess.run(
                ["bash", "-c", build_cmd],
                check=False,
                cwd=self.ros_dir,
                env=env,
                capture_output=True,
                text=True,
            )

            if result.returncode == 0:
                print("✅ Catkin build successful!")
            else:
                print(
                    f"⚠️  Catkin build completed with warnings (exit code: {result.returncode})"
                )
                if result.stderr:
                    print(f"Build stderr: {result.stderr}")

        except Exception as e:
            print(f"❌ Build failed with exception: {e}")

        # Update environment with workspace setup
        setup_file = self.ros_dir / "devel/setup.bash"
        if setup_file.exists():
            print("📦 Sourcing workspace setup...")

            # Source both ROS and workspace setup
            workspace_cmd = f"source {ros_setup} && source {setup_file} && env"

            try:
                result = subprocess.run(
                    ["bash", "-c", workspace_cmd],
                    capture_output=True,
                    text=True,
                    check=True,
                )

                # Update environment with workspace variables
                for line in result.stdout.split("\n"):
                    if "=" in line and not line.startswith("_"):
                        key, value = line.split("=", 1)
                        env[key] = value

                print("✅ Workspace environment configured")
                print(f"   ROS_PACKAGE_PATH: {env.get('ROS_PACKAGE_PATH', 'not set')}")

            except subprocess.CalledProcessError as e:
                print(f"⚠️  Failed to source workspace setup: {e}")
        else:
            print("⚠️  Workspace setup.bash not found, build may have failed")

        return env

    def _start_virtual_arduino(self):
        """Start virtual Arduino processes"""
        print("Checking for Arduino devices...")

        virtual_arduino_script = (
            self.ros_dir / "src/hydrus-software-stack/virtual_arduino.py"
        )

        for port in ["/dev/ttyACM0", "/dev/ttyACM1"]:
            try:
                process = subprocess.Popen(
                    ["python3", str(virtual_arduino_script), port],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.background_processes.append(process)
                print(f"Started virtual Arduino for {port}")
                time.sleep(2)
            except Exception as e:
                print(f"Failed to start virtual Arduino for {port}: {e}")

    def _compile_and_upload_arduino(self):
        """Compile and upload Arduino sketch"""
        arduino_dir = Path("/root/Arduino/libraries/embedded_arduino/Hydrus")
        if not arduino_dir.exists():
            print("Arduino directory not found, skipping Arduino upload")
            return

        print("Compiling Arduino sketch...")
        arduino_board = os.environ.get("ARDUINO_BOARD", "arduino:avr:mega")

        try:
            self._run_command(
                ["arduino-cli", "compile", "--fqbn", arduino_board, "Hydrus.ino"],
                check=False,
                cwd=arduino_dir,
            )
        except subprocess.CalledProcessError:
            print("Arduino compilation failed, continuing...")

        # Upload process with retry mechanism
        self._upload_arduino_with_retry(arduino_dir, arduino_board)

    def _upload_arduino_with_retry(self, arduino_dir: Path, arduino_board: str):
        """Upload Arduino sketch with retry mechanism"""
        print("Uploading Arduino sketch...")

        ports_to_try = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]
        max_attempts = 3
        upload_success = False

        for attempt in range(1, max_attempts + 1):
            if attempt > len(ports_to_try):
                break

            port = ports_to_try[attempt - 1]
            print(f"Upload attempt {attempt} of {max_attempts} using {port}")

            if not Path(port).exists():
                print(f"Port {port} does not exist, skipping this attempt")
                continue

            # Reset Arduino
            self._reset_arduino(port)

            try:
                result = self._run_command(
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
                    capture_output=True,
                    cwd=arduino_dir,
                )

                if result.returncode == 0:
                    print(f"✅ Upload successful to {port}!")
                    upload_success = True
                    # Save successful port
                    Path("/tmp/arduino_port.txt").write_text(f"Used port: {port}")
                    break
                else:
                    print(f"❌ Upload to {port} failed (exit code: {result.returncode})")
                    if attempt < max_attempts:
                        print("Trying next port in 2 seconds...")
                        time.sleep(2)
            except Exception as e:
                print(f"❌ Upload to {port} failed: {e}")

        if not upload_success:
            print("❌ All upload attempts failed. Continuing with virtual Arduino...")

    def _reset_arduino(self, port: str):
        """Reset Arduino by toggling DTR"""
        print(f"Resetting Arduino on {port}...")
        try:
            subprocess.run(
                ["stty", "-F", port, "hupcl"], check=False, capture_output=True
            )
            time.sleep(0.1)
            subprocess.run(
                ["stty", "-F", port, "-hupcl"], check=False, capture_output=True
            )
            time.sleep(0.5)
        except Exception:
            pass

    def _setup_serial_monitoring(self):
        """Setup serial monitoring for Arduino"""
        print("Setting up tmux sessions with Arduino monitoring...")

        # Make scripts executable
        scripts = [
            "scripts/setup_serial_monitor.py",
            "scripts/monitor_arduino_logs.py",
            "scripts/start_tmux_sessions.py",
        ]

        for script in scripts:
            script_path = self.ros_dir / "src/hydrus-software-stack" / script
            if script_path.exists():
                os.chmod(script_path, 0o755)

        # Start tmux sessions
        tmux_script = (
            self.ros_dir / "src/hydrus-software-stack/scripts/start_tmux_sessions.py"
        )
        if tmux_script.exists():
            try:
                subprocess.run([sys.executable, str(tmux_script)], check=False)
            except Exception as e:
                print(f"Failed to start tmux sessions: {e}")
        else:
            # Fallback to bash script
            tmux_script = (
                self.ros_dir / "src/hydrus-software-stack/start_tmux_sessions.sh"
            )
            if tmux_script.exists():
                os.chmod(tmux_script, 0o755)
                subprocess.run([str(tmux_script)], check=False)

    def _run_tests(self):
        """Run the test suite"""
        print("🧪 TEST mode detected - building and running tests")
        print("🔄 Setting up test environment...")
        os.environ["VOLUME"] = "false"

        env = self._setup_ros_environment()

        # Look for test script in the correct location within the workspace
        test_script = self.ros_dir / "src/hydrus-software-stack/scripts/run_tests.py"
        print(f"🔍 Looking for test script at: {test_script}")

        if test_script.exists():
            print(f"✅ Found test script at {test_script}")
            os.chmod(test_script, 0o755)
            print("🚀 Executing test suite...")
            print(f"💻 Command: {sys.executable} {test_script}")
            os.execve(sys.executable, [sys.executable, str(test_script)], env)
        else:
            print(f"❌ Test script not found at {test_script}")
            # Fallback to bash script
            test_script_bash = self.ros_dir / "src/hydrus-software-stack/run_tests.sh"
            print(f"🔍 Looking for bash test script at: {test_script_bash}")
            if test_script_bash.exists():
                print(f"✅ Found bash test script at {test_script_bash}")
                os.chmod(test_script_bash, 0o755)
                print("🚀 Executing bash test suite...")
                os.execve(str(test_script_bash), [str(test_script_bash)], env)
            else:
                print(f"❌ No test scripts found!")
                print("Available files in scripts directory:")
                scripts_dir = self.ros_dir / "src/hydrus-software-stack/scripts"
                if scripts_dir.exists():
                    for file in scripts_dir.iterdir():
                        print(f"  - {file}")
                else:
                    print(f"  Scripts directory {scripts_dir} does not exist!")
                sys.exit(1)

    def _start_rosbag_playback(self):
        """Start rosbag playback if enabled"""
        if not self.rosbag_playback:
            return

        print(
            "ROSBAG playback is enabled. Looking for rosbag files in /rosbags directory..."
        )

        rosbags_dir = Path("/rosbags")
        if rosbags_dir.exists():
            bag_files = list(rosbags_dir.glob("*.bag"))
            if bag_files:
                print("Found the following rosbag files:")
                for bag in bag_files:
                    print(f"  - {bag}")

                # Get the most recent rosbag file
                most_recent = max(bag_files, key=lambda x: x.stat().st_mtime)
                print(f"Automatically selecting most recent rosbag: {most_recent}")

                # Start rosbag play in background
                print("Playing rosbag in loop mode...")
                process = subprocess.Popen(
                    ["rosbag", "play", str(most_recent), "--loop"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.background_processes.append(process)
                print("Rosbag playback started in background.")
            else:
                print(
                    "No rosbag files found in /rosbags directory. Skipping rosbag playback."
                )
        else:
            print("/rosbags directory not found. Skipping rosbag playback.")

    def _start_rviz(self):
        """Start RViz if enabled"""
        if not self.rviz:
            return

        print("Launching RViz with the specified configuration...")
        env = self._setup_ros_environment()

        # Source workspace setup
        setup_file = self.ros_dir / "devel/setup.bash"
        if setup_file.exists():
            env[
                "ROS_PACKAGE_PATH"
            ] = f"{self.ros_dir}/src:{env.get('ROS_PACKAGE_PATH', '')}"

        rviz_config_path = (
            self.ros_dir
            / "src/hydrus-software-stack/autonomy/config"
            / self.rviz_config
        )

        try:
            process = subprocess.Popen(
                ["rviz", "-d", str(rviz_config_path)],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.background_processes.append(process)
            print("RViz started in background.")
        except Exception as e:
            print(f"Failed to start RViz: {e}")

    def _cleanup(self, signum=None, frame=None):
        """Cleanup background processes"""
        print("\nCleaning up background processes...")
        for process in self.background_processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except:
                try:
                    process.kill()
                except:
                    pass

    def main(self):
        """Main execution function"""
        # Setup signal handlers for cleanup
        signal.signal(signal.SIGINT, self._cleanup)
        signal.signal(signal.SIGTERM, self._cleanup)

        try:
            # Build workspace (skip if no_build is enabled)
            if self.no_build:
                # Setup ROS environment
                env = self._setup_ros_environment()
                # Clean simulator on ARM
                print("🚫 NO_BUILD flag is set - skipping catkin workspace build")
                print("🐛 This is useful for debugging the entrypoint script")
                # Still need to source existing workspace if it exists
                setup_file = self.ros_dir / "devel/setup.bash"
                if setup_file.exists():
                    print("📦 Found existing workspace setup, sourcing it...")
                    ros_setup = f"/opt/ros/{self.ros_distro}/setup.bash"
                    workspace_cmd = f"source {ros_setup} && source {setup_file} && env"

                    try:
                        result = subprocess.run(
                            ["bash", "-c", workspace_cmd],
                            capture_output=True,
                            text=True,
                            check=True,
                        )

                        # Update environment with workspace variables
                        for line in result.stdout.split("\n"):
                            if "=" in line and not line.startswith("_"):
                                key, value = line.split("=", 1)
                                env[key] = value

                        print("✅ Existing workspace environment configured")

                    except subprocess.CalledProcessError as e:
                        print(f"⚠️  Failed to source existing workspace: {e}")
                else:
                    print("📦 No existing workspace found, using base ROS environment")
            else:
                env = self._build_workspace()
                time.sleep(2)

            # DEPLOY SECTION
            if self.deploy:
                print("Starting rosserial_python node...")
                time.sleep(1)

                # Start virtual Arduino
                self._start_virtual_arduino()

                # Compile and upload Arduino
                self._compile_and_upload_arduino()

                # Setup serial monitoring and tmux sessions
                self._setup_serial_monitoring()

                print("DEPLOY setup completed successfully")
            else:
                print(
                    "Deploy is not set or is set to false. Skipping deployment setup."
                )

            # TEST SECTION
            if self.test:
                self._run_tests()
                # This won't return due to exec

            # Start rosbag playback
            self._start_rosbag_playback()

            # Start RViz
            self._start_rviz()

            # Keep container running if not in test mode
            if not self.test:
                print("Setup complete. Keeping container running...")
                if self.no_build:
                    print(
                        "🐛 Container is ready for debugging - you can now attach VS Code"
                    )
                    print(
                        "   The ros-entrypoint.py script will remain running for debugging"
                    )
                try:
                    # Keep the container running
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    pass

        finally:
            self._cleanup()


if __name__ == "__main__":
    manager = HydrusROSManager()
    manager.main()
