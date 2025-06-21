#!/usr/bin/env python3
"""
Setup script for the SAM2 Data Engine
"""

import os
import subprocess
import sys
from pathlib import Path


def install_requirements():
    """Install required packages"""
    requirements_file = Path(__file__).parent / "requirements.txt"

    if not requirements_file.exists():
        print("Requirements file not found!")
        return False

    try:
        print("Installing requirements...")
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "-r", str(requirements_file)]
        )
        print("Requirements installed successfully!")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Failed to install requirements: {e}")
        return False


def download_sam_models():
    """Download SAM2 models"""
    try:
        print("Downloading SAM2 models...")
        # This will download the model when first used
        from ultralytics import SAM

        # Try to load different SAM2 models
        models = ["sam2_b.pt", "sam2_l.pt", "sam2_s.pt"]

        for model_name in models:
            try:
                print(f"Downloading {model_name}...")
                sam = SAM(model_name)
                print(f"✓ {model_name} downloaded successfully")
                break  # Download at least one model
            except Exception as e:
                print(f"Failed to download {model_name}: {e}")
                continue

        return True

    except ImportError:
        print("Ultralytics not installed. Please install requirements first.")
        return False
    except Exception as e:
        print(f"Failed to download models: {e}")
        return False


def create_directories():
    """Create necessary directories"""
    base_dir = Path(__file__).parent
    directories = [
        "cache",
        "cache/frames",
        "cache/masks",
        "cache/features",
        "projects",
        "exports",
    ]

    for dir_name in directories:
        dir_path = base_dir / dir_name
        dir_path.mkdir(parents=True, exist_ok=True)
        print(f"Created directory: {dir_path}")


def check_system_requirements():
    """Check system requirements"""
    print("Checking system requirements...")

    # Check Python version
    if sys.version_info < (3, 8):
        print("Error: Python 3.8 or higher is required")
        return False

    print(f"✓ Python {sys.version_info.major}.{sys.version_info.minor}")

    # Check for CUDA (optional)
    try:
        import torch

        if torch.cuda.is_available():
            print(f"✓ CUDA available: {torch.cuda.get_device_name()}")
        else:
            print("⚠ CUDA not available, will use CPU (slower)")
    except ImportError:
        print("⚠ PyTorch not installed yet")

    return True


def main():
    """Main setup function"""
    print("=" * 50)
    print("SAM2 Data Engine Setup")
    print("=" * 50)

    # Check system requirements
    if not check_system_requirements():
        return False

    # Install requirements
    if not install_requirements():
        return False

    # Create directories
    create_directories()

    # Download models
    if not download_sam_models():
        print("⚠ Model download failed, but you can try again later")

    print("\n" + "=" * 50)
    print("Setup completed!")
    print("You can now run the data engine with: python main.py")
    print("=" * 50)

    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
