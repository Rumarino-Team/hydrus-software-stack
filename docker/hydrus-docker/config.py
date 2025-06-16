"""
Configuration management for Hydrus Docker deployment
"""

# Available configuration options
AVAILABLE_OPTIONS = [
    "volume",
    "force_cpu",
    "force_jetson",
    "rviz",
    "zed_option",
    "rosbag_playback",
    "debug_arduino",
    "vscode",
    "install_vscode_extensions",
    "test",
    "no_build",
    "tmux_sessions",
    "arduino_compile",
    "virtual_arduino",
]

# Predefined configuration groups - only list enabled options
CONFIGURATION_GROUPS = {
    "test": {
        "description": "Automated testing configuration",
        "enabled_options": [
            "force_cpu",
            "test",
        ],
    },
    "development": {
        "description": "Development configuration with volumes and debugging",
        "enabled_options": [
            "volume",
            "force_cpu",
            "rviz",
            "rosbag_playback",
            "debug_arduino",
        ],
    },
    "production": {
        "description": "Production deployment configuration",
        "enabled_options": [
            "zed_option",
        ],
    },
    "simulation": {
        "description": "Simulation and visualization configuration",
        "enabled_options": [
            "volume",
            "force_cpu",
            "rviz",
            "rosbag_playback",
            "vscode",
            "install_vscode_extensions",
        ],
    },
    "vscode-debugging": {
        "description": "VS Code debugging configuration - no build, keeps container running for debugging",
        "enabled_options": [
            "volume",
            "force_cpu",
            "vscode",
            "install_vscode_extensions",
            "no_build",
        ],
    },
    "deploy": {
        "description": "Full deployment with all hardware components (tmux, Arduino, virtual Arduino)",
        "enabled_options": [
            "tmux_sessions",
            "arduino_compile",
            "virtual_arduino",
        ],
    },
}


class ConfigurationManager:
    """Manages configuration groups and settings"""

    @staticmethod
    def get_available_groups():
        """Get list of available configuration groups"""
        return list(CONFIGURATION_GROUPS.keys())

    @staticmethod
    def get_group_info(group_name: str):
        """Get information about a specific configuration group"""
        return CONFIGURATION_GROUPS.get(group_name)

    @staticmethod
    def apply_configuration_group(group_name: str, base_config: dict) -> dict:
        """Apply a predefined configuration group to the base config"""
        if group_name not in CONFIGURATION_GROUPS:
            raise ValueError(f"Unknown configuration group: {group_name}")

        group = CONFIGURATION_GROUPS[group_name]
        print(f"📋 Applying configuration group: {group_name}")
        print(f"   Description: {group['description']}")

        # Start with all options set to False
        config = {option: False for option in AVAILABLE_OPTIONS}

        # Override with base config values
        config.update(base_config)

        # Enable options specified in the group
        for option in group["enabled_options"]:
            config[option] = True

        print(f"   Enabled options: {', '.join(group['enabled_options'])}")

        return config

    @staticmethod
    def create_base_config(args) -> dict:
        """Create base configuration from command line arguments"""
        return {
            "volume": args.volume,
            "force_cpu": args.force_cpu,
            "force_jetson": args.force_jetson,
            "rviz": args.rviz,
            "zed_option": args.zed,
            "rosbag_playback": args.rosbag,
            "debug_arduino": args.debug_arduino,
            "vscode": args.vscode,
            "install_vscode_extensions": args.install_vscode_extensions,
            "no_build": args.no_build,
            "tmux_sessions": getattr(args, "tmux_sessions", False),
            "arduino_compile": getattr(args, "arduino_compile", False),
            "virtual_arduino": getattr(args, "virtual_arduino", False),
        }

    @staticmethod
    def apply_argument_overrides(config: dict, args) -> dict:
        """Apply argument overrides (negation arguments) to configuration"""
        if args.no_volume:
            config["volume"] = False
        if args.no_rviz:
            config["rviz"] = False
        if args.no_zed:
            config["zed_option"] = False
        if args.no_rosbag:
            config["rosbag_playback"] = False
        if args.no_debug_arduino:
            config["debug_arduino"] = False
        if args.no_vscode:
            config["vscode"] = False

        return config
