"""
Command line interface for Hydrus Docker deployment
"""

import argparse
from config import CONFIGURATION_GROUPS


class CLIParser:
    """Handles command line argument parsing"""
    
    @staticmethod
    def create_parser():
        """Create and configure the argument parser"""
        parser = argparse.ArgumentParser(
            description="Hydrus Docker Deployment Manager",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=f"""
Configuration Groups:
{chr(10).join([f"  --{name}: {info['description']}" for name, info in CONFIGURATION_GROUPS.items()])}

Examples:
  # Use predefined configurations
  python3 run_docker.py --test
  python3 run_docker.py --development
  python3 run_docker.py --production
  python3 run_docker.py --simulation

  # Basic deployment with auto-detection
  python3 run_docker.py

  # CPU-only deployment with RViz
  python3 run_docker.py --force-cpu --rviz

  # CUDA deployment with ZED camera and rosbag playback
  python3 run_docker.py --cuda --zed --rosbag --download-rosbag

  # Deployment with VS Code integration
  python3 run_docker.py --vscode --install-vscode-extensions

  # Execute into running container
  python3 run_docker.py --exec

  # Destroy all containers
  python3 run_docker.py --destroy

  # Override configuration group settings
  python3 run_docker.py --development --force-cpu --no-debug-arduino
            """
        )
        
        # Configuration group arguments
        group_parser = parser.add_argument_group('Configuration Groups')
        for group_name, group_info in CONFIGURATION_GROUPS.items():
            group_parser.add_argument(f"--{group_name}", action="store_true", 
                                    help=group_info['description'])
        
        # Deployment type arguments
        deployment_group = parser.add_argument_group('Deployment Type')
        deployment_group.add_argument("--force-cpu", action="store_true", help="Force CPU deployment")
        deployment_group.add_argument("--cuda", action="store_true", help="Force CUDA/GPU deployment")
        deployment_group.add_argument("--force-jetson", action="store_true", help="Force Jetson deployment")
        
        # Configuration arguments
        config_group = parser.add_argument_group('Configuration')
        config_group.add_argument("--deploy", action="store_true", help="Enable deployment mode")
        config_group.add_argument("--volume", action="store_true", help="Use volume mounting")
        config_group.add_argument("--rviz", action="store_true", help="Enable RViz")
        config_group.add_argument("--zed", action="store_true", help="Enable ZED camera display in RViz")
        config_group.add_argument("--rosbag", action="store_true", help="Enable rosbag playback")
        config_group.add_argument("--download-rosbag", action="store_true", help="Auto-download rosbag files if missing")
        config_group.add_argument("--debug-arduino", action="store_true", help="Enable Arduino serial debugging")
        
        # Negation arguments for overriding configuration groups
        config_group.add_argument("--no-deploy", action="store_true", help="Disable deployment mode")
        config_group.add_argument("--no-volume", action="store_true", help="Disable volume mounting")
        config_group.add_argument("--no-rviz", action="store_true", help="Disable RViz")
        config_group.add_argument("--no-zed", action="store_true", help="Disable ZED camera display")
        config_group.add_argument("--no-rosbag", action="store_true", help="Disable rosbag playback")
        config_group.add_argument("--no-debug-arduino", action="store_true", help="Disable Arduino debugging")
        
        # VS Code integration arguments
        vscode_group = parser.add_argument_group('VS Code Integration')
        vscode_group.add_argument("--vscode", action="store_true", help="Open VS Code and connect to container")
        vscode_group.add_argument("--install-vscode-extensions", action="store_true", help="Auto-install missing VS Code extensions")
        vscode_group.add_argument("--no-vscode", action="store_true", help="Disable VS Code integration")
        
        # Action arguments
        action_group = parser.add_argument_group('Actions')
        action_group.add_argument("--exec", action="store_true", help="Execute into running container and replace current shell")
        action_group.add_argument("--destroy", action="store_true", help="Destroy containers and cleanup Docker resources")
        
        return parser
    
    @staticmethod
    def validate_arguments(args):
        """Validate parsed arguments"""
        # Check for conflicting deployment types
        deployment_flags = [args.force_cpu, args.cuda, args.force_jetson]
        if sum(deployment_flags) > 1:
            raise ValueError("Cannot specify multiple deployment types (--force-cpu, --cuda, --force-jetson)")
        
        # Check for multiple configuration groups
        active_groups = [group for group in CONFIGURATION_GROUPS.keys() if getattr(args, group, False)]
        if len(active_groups) > 1:
            raise ValueError(f"Multiple configuration groups specified: {', '.join(active_groups)}. Please specify only one.")
        
        return True