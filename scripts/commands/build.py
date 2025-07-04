"""
Build command module for Hydrus CLI
Handles workspace building operations
"""

import os
import subprocess
import sys
from pathlib import Path


def run_build(args):
    """Build the catkin workspace"""
    print("🔨 Building catkin workspace...")

    # Find workspace
    script_dir = Path(__file__).parent.parent.absolute()
    hydrus_root = script_dir.parent

    # Determine workspace type
    volume_path = Path("/home/catkin_ws")
    container_path = Path("/catkin_ws")

    if volume_path.exists() and (volume_path / "src").exists():
        workspace_dir = volume_path
    elif container_path.exists() and (container_path / "src").exists():
        workspace_dir = container_path
    else:
        print("❌ Could not find catkin workspace")
        return False

    # Setup environment
    env = os.environ.copy()
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

    # Clean if requested
    if args.clean:
        print("🧹 Cleaning workspace...")
        clean_cmd = f"cd {workspace_dir} && catkin_make clean"
        subprocess.run(["bash", "-c", clean_cmd], check=False)

    # Build
    if not args.no_build:
        source_cmd = f"source /opt/ros/{env.get('ROS_DISTRO', 'noetic')}/setup.bash"
        build_cmd = f"{source_cmd} && cd {workspace_dir} && catkin_make"

        result = subprocess.run(["bash", "-c", build_cmd], env=env, check=False)

        if result.returncode == 0:
            print("✅ Workspace build completed successfully")
            return True
        else:
            print("❌ Workspace build failed")
            return False

    return True


def register_subcommand(subparsers):
    """Register the build subcommand with the main parser"""
    build_parser = subparsers.add_parser(
        "build",
        help="Build the ROS catkin workspace",
        description="Build the ROS catkin workspace with optional cleaning.",
    )

    build_parser.add_argument(
        "--clean", action="store_true", help="Clean workspace before building"
    )
    build_parser.add_argument(
        "--no-build", action="store_true", help="Skip the actual build step"
    )

    # Set the function to call
    build_parser.set_defaults(func=run_build)
