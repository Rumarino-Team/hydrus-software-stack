"""
Configuration management for Hydrus Docker deployment
"""

# Predefined configuration groups for common use cases
CONFIGURATION_GROUPS = {
    'test': {
        'description': 'Automated testing configuration',
        'config': {
            'deploy': False,
            'volume': False,
            'force_cpu': True,
            'rviz': False,
            'zed_option': False,
            'rosbag_playback': True,
            'debug_arduino': False,
            'vscode': False,
            'install_vscode_extensions': False,
            'test': True  # ✅ Add this to set TEST=true in container
        }
    },
    'development': {
        'description': 'Development configuration with volumes and debugging',
        'config': {
            'deploy': True,
            'volume': True,
            'force_cpu': True,  # Auto-detect
            'rviz': True,
            'zed_option': False,
            'rosbag_playback': True,
            'debug_arduino': True,
            'vscode': False,  # Disabled for development as requested
            'install_vscode_extensions': False
        }
    },
    'production': {
        'description': 'Production deployment configuration',
        'config': {
            'deploy': True,
            'volume': False,
            'force_cpu': False,  # Auto-detect
            'rviz': False,
            'zed_option': True,
            'rosbag_playback': False,
            'debug_arduino': False,
            'vscode': False,
            'install_vscode_extensions': False
        }
    },
    'simulation': {
        'description': 'Simulation and visualization configuration',
        'config': {
            'deploy': False,
            'volume': True,
            'force_cpu': True,
            'rviz': True,
            'zed_option': False,
            'rosbag_playback': True,
            'debug_arduino': True,
            'vscode': True,
            'install_vscode_extensions': True
        }
    }
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
        
        # Apply group configuration to base config
        updated_config = base_config.copy()
        updated_config.update(group['config'])
        
        print(f"   Configuration applied:")
        for key, value in group['config'].items():
            print(f"     {key}: {value}")
        
        return updated_config
    
    @staticmethod
    def create_base_config(args) -> dict:
        """Create base configuration from command line arguments"""
        return {
            'deploy': args.deploy,
            'volume': args.volume,
            'force_cpu': args.force_cpu,
            'force_jetson': args.force_jetson,
            'rviz': args.rviz,
            'zed_option': args.zed,
            'rosbag_playback': args.rosbag,
            'debug_arduino': args.debug_arduino,
            'vscode': args.vscode,
            'install_vscode_extensions': args.install_vscode_extensions
        }
    
    @staticmethod
    def apply_argument_overrides(config: dict, args) -> dict:
        """Apply argument overrides (negation arguments) to configuration"""
        if args.no_deploy:
            config['deploy'] = False
        if args.no_volume:
            config['volume'] = False
        if args.no_rviz:
            config['rviz'] = False
        if args.no_zed:
            config['zed_option'] = False
        if args.no_rosbag:
            config['rosbag_playback'] = False
        if args.no_debug_arduino:
            config['debug_arduino'] = False
        if args.no_vscode:
            config['vscode'] = False
        
        return config