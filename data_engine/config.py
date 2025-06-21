#!/usr/bin/env python3
"""
Configuration management for the Data Engine
"""

import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Optional


@dataclass
class SAMConfig:
    """SAM2 model configuration"""

    model_name: str = "sam2_b.pt"
    device: str = "auto"
    multimask_output: bool = False
    cache_features: bool = True
    batch_size: int = 1


@dataclass
class UIConfig:
    """UI configuration"""

    window_width: int = 1400
    window_height: int = 900
    frame_cache_size: int = 100
    auto_save_interval: int = 300  # seconds
    default_point_size: int = 5
    overlay_alpha: float = 0.5


@dataclass
class ExportConfig:
    """Export configuration"""

    export_format: str = "yolo"  # yolo, coco, voc
    image_format: str = "jpg"
    compression_quality: int = 95
    include_empty_frames: bool = False
    validate_annotations: bool = True


@dataclass
class CacheConfig:
    """Cache configuration"""

    max_cache_size_gb: float = 5.0
    auto_cleanup: bool = True
    cleanup_interval_hours: int = 24
    keep_recent_projects: int = 5


@dataclass
class DataEngineConfig:
    """Main configuration class"""

    sam: SAMConfig
    ui: UIConfig
    export: ExportConfig
    cache: CacheConfig
    project_name: str = "untitled"
    last_video_path: str = ""
    last_output_dir: str = ""
    classes: Dict[int, str] = None

    def __post_init__(self):
        if self.classes is None:
            self.classes = {}


class ConfigManager:
    """Configuration manager for the Data Engine"""

    def __init__(self, config_dir: str = None):
        if config_dir is None:
            config_dir = Path.home() / ".data_engine"

        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(exist_ok=True)

        self.config_file = self.config_dir / "config.json"
        self.projects_dir = self.config_dir / "projects"
        self.projects_dir.mkdir(exist_ok=True)

        self._config = self._load_config()

    def _load_config(self) -> DataEngineConfig:
        """Load configuration from file"""
        if self.config_file.exists():
            try:
                with open(self.config_file, "r") as f:
                    data = json.load(f)

                # Convert nested dictionaries back to dataclasses
                sam_config = SAMConfig(**data.get("sam", {}))
                ui_config = UIConfig(**data.get("ui", {}))
                export_config = ExportConfig(**data.get("export", {}))
                cache_config = CacheConfig(**data.get("cache", {}))

                return DataEngineConfig(
                    sam=sam_config,
                    ui=ui_config,
                    export=export_config,
                    cache=cache_config,
                    project_name=data.get("project_name", "untitled"),
                    last_video_path=data.get("last_video_path", ""),
                    last_output_dir=data.get("last_output_dir", ""),
                    classes=data.get("classes", {}),
                )
            except Exception as e:
                print(f"Error loading config: {e}, using defaults")

        # Return default configuration
        return DataEngineConfig(
            sam=SAMConfig(), ui=UIConfig(), export=ExportConfig(), cache=CacheConfig()
        )

    def save_config(self):
        """Save configuration to file"""
        try:
            # Convert dataclasses to dictionaries
            config_dict = {
                "sam": asdict(self._config.sam),
                "ui": asdict(self._config.ui),
                "export": asdict(self._config.export),
                "cache": asdict(self._config.cache),
                "project_name": self._config.project_name,
                "last_video_path": self._config.last_video_path,
                "last_output_dir": self._config.last_output_dir,
                "classes": self._config.classes,
            }

            with open(self.config_file, "w") as f:
                json.dump(config_dict, f, indent=2)

        except Exception as e:
            print(f"Error saving config: {e}")

    @property
    def config(self) -> DataEngineConfig:
        """Get current configuration"""
        return self._config

    def update_sam_config(self, **kwargs):
        """Update SAM configuration"""
        for key, value in kwargs.items():
            if hasattr(self._config.sam, key):
                setattr(self._config.sam, key, value)
        self.save_config()

    def update_ui_config(self, **kwargs):
        """Update UI configuration"""
        for key, value in kwargs.items():
            if hasattr(self._config.ui, key):
                setattr(self._config.ui, key, value)
        self.save_config()

    def update_export_config(self, **kwargs):
        """Update export configuration"""
        for key, value in kwargs.items():
            if hasattr(self._config.export, key):
                setattr(self._config.export, key, value)
        self.save_config()

    def update_cache_config(self, **kwargs):
        """Update cache configuration"""
        for key, value in kwargs.items():
            if hasattr(self._config.cache, key):
                setattr(self._config.cache, key, value)
        self.save_config()

    def set_last_paths(self, video_path: str = None, output_dir: str = None):
        """Update last used paths"""
        if video_path:
            self._config.last_video_path = video_path
        if output_dir:
            self._config.last_output_dir = output_dir
        self.save_config()

    def save_project(self, project_name: str, project_data: Dict[str, Any]):
        """Save project data"""
        project_file = self.projects_dir / f"{project_name}.json"
        try:
            with open(project_file, "w") as f:
                json.dump(project_data, f, indent=2)
        except Exception as e:
            print(f"Error saving project: {e}")

    def load_project(self, project_name: str) -> Optional[Dict[str, Any]]:
        """Load project data"""
        project_file = self.projects_dir / f"{project_name}.json"
        if project_file.exists():
            try:
                with open(project_file, "r") as f:
                    return json.load(f)
            except Exception as e:
                print(f"Error loading project: {e}")
        return None

    def list_projects(self) -> list:
        """List available projects"""
        projects = []
        for project_file in self.projects_dir.glob("*.json"):
            projects.append(project_file.stem)
        return sorted(projects)

    def delete_project(self, project_name: str) -> bool:
        """Delete a project"""
        project_file = self.projects_dir / f"{project_name}.json"
        try:
            if project_file.exists():
                project_file.unlink()
                return True
        except Exception as e:
            print(f"Error deleting project: {e}")
        return False

    def get_cache_dir(self) -> Path:
        """Get cache directory"""
        cache_dir = self.config_dir / "cache"
        cache_dir.mkdir(exist_ok=True)
        return cache_dir

    def cleanup_cache(self, max_size_gb: float = None):
        """Clean up cache directory"""
        if max_size_gb is None:
            max_size_gb = self._config.cache.max_cache_size_gb

        cache_dir = self.get_cache_dir()
        if not cache_dir.exists():
            return

        # Calculate current cache size
        total_size = 0
        cache_files = []

        for file_path in cache_dir.rglob("*"):
            if file_path.is_file():
                size = file_path.stat().st_size
                total_size += size
                cache_files.append((file_path, size, file_path.stat().st_mtime))

        # Convert to GB
        total_size_gb = total_size / (1024**3)

        if total_size_gb > max_size_gb:
            # Sort by modification time (oldest first)
            cache_files.sort(key=lambda x: x[2])

            # Remove oldest files until under limit
            target_size = max_size_gb * 0.8  # Remove to 80% of limit
            current_size_gb = total_size_gb

            for file_path, size, _ in cache_files:
                if current_size_gb <= target_size:
                    break
                try:
                    file_path.unlink()
                    current_size_gb -= size / (1024**3)
                except OSError:
                    pass

    def get_model_cache_dir(self) -> Path:
        """Get model cache directory"""
        model_dir = self.get_cache_dir() / "models"
        model_dir.mkdir(exist_ok=True)
        return model_dir

    def export_config(self, export_path: str):
        """Export configuration to a file"""
        try:
            config_dict = {
                "sam": asdict(self._config.sam),
                "ui": asdict(self._config.ui),
                "export": asdict(self._config.export),
                "cache": asdict(self._config.cache),
                "classes": self._config.classes,
            }

            with open(export_path, "w") as f:
                json.dump(config_dict, f, indent=2)
            return True
        except Exception as e:
            print(f"Error exporting config: {e}")
            return False

    def import_config(self, import_path: str) -> bool:
        """Import configuration from a file"""
        try:
            with open(import_path, "r") as f:
                config_dict = json.load(f)

            # Update configuration
            if "sam" in config_dict:
                self.update_sam_config(**config_dict["sam"])
            if "ui" in config_dict:
                self.update_ui_config(**config_dict["ui"])
            if "export" in config_dict:
                self.update_export_config(**config_dict["export"])
            if "cache" in config_dict:
                self.update_cache_config(**config_dict["cache"])
            if "classes" in config_dict:
                self._config.classes = config_dict["classes"]
                self.save_config()

            return True
        except Exception as e:
            print(f"Error importing config: {e}")
            return False


# Global configuration manager instance
_config_manager = None


def get_config_manager() -> ConfigManager:
    """Get global configuration manager"""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigManager()
    return _config_manager


def get_config() -> DataEngineConfig:
    """Get current configuration"""
    return get_config_manager().config
