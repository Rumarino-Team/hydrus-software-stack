#!/usr/bin/env python3
"""
Example script demonstrating SAM2 Data Engine features
"""

import sys
from pathlib import Path

import numpy as np

# Add data_engine to path
sys.path.insert(0, str(Path(__file__).parent))


def create_sample_data():
    """Create some sample data for demonstration"""
    print("Creating sample data...")

    # Create a simple test image
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # Add some shapes for segmentation
    import cv2

    # Draw a circle
    cv2.circle(test_image, (200, 200), 50, (255, 0, 0), -1)

    # Draw a rectangle
    cv2.rectangle(test_image, (400, 150), (500, 250), (0, 255, 0), -1)

    # Draw a triangle
    points = np.array([[300, 350], [350, 300], [400, 350]], np.int32)
    cv2.fillPoly(test_image, [points], (0, 0, 255))

    return test_image


def demo_sam_processor():
    """Demonstrate SAM processor functionality"""
    print("Testing SAM processor...")

    try:
        from sam_processor import SAM2Processor

        # Initialize processor
        processor = SAM2Processor()

        if not processor.is_loaded:
            print("⚠ SAM model not loaded - this is expected without dependencies")
            return False

        # Test with sample image
        test_image = create_sample_data()

        # Test point segmentation
        points = np.array([[200, 200], [400, 200]])  # Points in the shapes
        labels = np.array([1, 1])  # Both positive

        result = processor.segment_with_points(test_image, points, labels)

        if result["success"]:
            print("✓ SAM segmentation successful")
        else:
            print(f"✗ SAM segmentation failed: {result.get('error', 'Unknown error')}")

        return True

    except ImportError as e:
        print(f"⚠ SAM processor import failed: {e}")
        return False


def demo_utils():
    """Demonstrate utility functions"""
    print("Testing utility functions...")

    try:
        from utils import create_directory_structure, mask_to_polygon, polygon_to_mask

        # Test mask to polygon conversion
        test_mask = np.zeros((100, 100), dtype=np.uint8)
        test_mask[30:70, 30:70] = 255  # Square mask

        polygon = mask_to_polygon(test_mask)
        print(f"✓ Converted mask to polygon with {len(polygon)//2} points")

        # Test polygon to mask conversion
        reconstructed_mask = polygon_to_mask(polygon, 100, 100)
        print(f"✓ Reconstructed mask shape: {reconstructed_mask.shape}")

        # Test directory structure creation
        test_dir = Path(__file__).parent / "test_output"
        directories = create_directory_structure(str(test_dir))
        print(f"✓ Created {len(directories)} directories")

        # Cleanup test directory
        import shutil

        if test_dir.exists():
            shutil.rmtree(test_dir)

        return True

    except ImportError as e:
        print(f"⚠ Utils import failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Utils test failed: {e}")
        return False


def demo_config():
    """Demonstrate configuration management"""
    print("Testing configuration...")

    try:
        from config import ConfigManager

        # Create test config manager
        test_config_dir = Path(__file__).parent / "test_config"
        config_manager = ConfigManager(str(test_config_dir))

        # Test configuration access
        config = config_manager.config
        print(f"✓ Loaded config with SAM model: {config.sam.model_name}")

        # Test configuration updates
        config_manager.update_sam_config(model_name="sam2_l.pt")
        print("✓ Updated SAM configuration")

        # Test project management
        test_project_data = {
            "video_path": "/test/video.mp4",
            "annotations": {"frame_1": ["object_1"]},
            "classes": {0: "test_class"},
        }

        config_manager.save_project("test_project", test_project_data)
        loaded_project = config_manager.load_project("test_project")

        if loaded_project:
            print("✓ Project save/load successful")
        else:
            print("✗ Project save/load failed")

        # Cleanup test config
        import shutil

        if test_config_dir.exists():
            shutil.rmtree(test_config_dir)

        return True

    except ImportError as e:
        print(f"⚠ Config import failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Config test failed: {e}")
        return False


def main():
    """Run all demonstrations"""
    print("SAM2 Data Engine - Example and Test Script")
    print("=" * 50)

    tests = [
        ("Utility Functions", demo_utils),
        ("Configuration Management", demo_config),
        ("SAM Processor", demo_sam_processor),
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n--- {test_name} ---")
        try:
            if test_func():
                passed += 1
                print(f"✓ {test_name} passed")
            else:
                print(f"⚠ {test_name} completed with warnings")
        except Exception as e:
            print(f"✗ {test_name} failed: {e}")

    print(f"\n" + "=" * 50)
    print(f"Tests completed: {passed}/{total} passed")

    if passed == total:
        print("✓ All tests passed! The Data Engine should work correctly.")
    elif passed > 0:
        print(
            "⚠ Some tests passed. Install missing dependencies for full functionality."
        )
    else:
        print("✗ Most tests failed. Please install dependencies and check setup.")

    print("\nTo install dependencies:")
    print("pip install -r requirements.txt")
    print("\nTo run the full application:")
    print("python run.py")

    return 0 if passed > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
