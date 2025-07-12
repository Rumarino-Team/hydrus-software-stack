#!/usr/bin/env python3
"""
Hocker - Hydrus Docker Deployment Tool (Docker-focused only)
Pure Docker management - delegates all Hydrus software control to hydrus-cli
"""

import argparse
import sys

from docker_manager import DockerManager
from system_detector import SystemDetector, SystemType


class HockerDockerDeployment:
    """Pure Docker orchestrator - focused only on container management"""

    def __init__(self):
        # Initialize managers
        self.docker_manager = DockerManager()

    def setup_argument_parser(self):
        """Set up argument parser for Docker deployment options"""
        parser = argparse.ArgumentParser(
            description="Hocker - Hydrus Docker Deployment Tool",
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )

        # Docker configuration options
        parser.add_argument(
            "--force-cpu", action="store_true", help="Force CPU-only deployment"
        )
        parser.add_argument(
            "--force-jetson", action="store_true", help="Force Jetson deployment"
        )
        parser.add_argument(
            "--vscode", action="store_true", help="Enable VS Code integration"
        )
        parser.add_argument(
            "--install-vscode-extensions",
            action="store_true",
            help="Install VS Code extensions",
        )

        # Container actions
        parser.add_argument(
            "--exec", type=str, metavar="COMMAND", help="Execute command in container"
        )
        parser.add_argument(
            "--it", action="store_true", help="Interactive mode for exec"
        )
        parser.add_argument("--destroy", action="store_true", help="Destroy containers")
        parser.add_argument(
            "--detach",
            "-d",
            action="store_true",
            help="Run containers in detached mode",
        )

        return parser

    def execute_action(self, args):
        dict_args = {k: v for k, v in vars(args).items() if v is not None}

        # Determine system type and force type based on flags
        system_type = None
        force_type = None

        if args.force_cpu:
            system_type = SystemType.CPU
            force_type = "cpu"
        elif args.force_jetson:
            system_type = SystemType.JETSON_TX2
            force_type = "jetson"

        compose_file = SystemDetector.determine_compose_file(system_type)

        if args.exec:
            # Parse the command properly - split into list for exec
            command_parts = args.exec.split()
            del dict_args["exec"]
            self.docker_manager.exec_into_container(
                compose_file,
                interactive=args.it,
                command=command_parts,
                force_type=force_type,
            )
        elif args.destroy:
            self.docker_manager.destroy_containers(compose_file)
        else:
            self.docker_manager.run_docker_compose(compose_file, dict_args)

    def main(self):
        """Main execution function"""
        try:
            # Parse command line arguments
            parser = self.setup_argument_parser()
            args = parser.parse_args()
            # Execute the requested action
            self.execute_action(args)

        except ValueError as e:
            print(f"❌ Error: {e}")
            sys.exit(1)
        except KeyboardInterrupt:
            print("\n🛑 Operation cancelled by user")
            sys.exit(1)
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
            sys.exit(1)


if __name__ == "__main__":
    deployment = HockerDockerDeployment()
    deployment.main()
