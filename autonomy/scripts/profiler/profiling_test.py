#!/usr/bin/env python3
"""
Simple test of the profiling decorators without ROS dependencies
This allows testing the profiling functionality independently
"""

import os
import sys
import time

import numpy as np

# Add the current directory to path
sys.path.append(os.path.dirname(__file__))

from profiling_decorators import (
    FunctionProfiler,
    ProfileManager,
    export_profiling_data,
    profile_function,
    profile_method,
)

# Initialize profiling
ProfileManager.get_instance().set_node_name("profiling_test")


@profile_function("image_processing", "cv")
def process_image(image):
    """Mock image processing function"""
    with FunctionProfiler("cv.blur"):
        # Simulate Gaussian blur
        time.sleep(0.01)  # Simulate processing time

    with FunctionProfiler("cv.threshold"):
        # Simulate thresholding
        time.sleep(0.005)

    return True


@profile_function("detection", "cv")
def detect_objects(image):
    """Mock object detection"""
    with FunctionProfiler("cv.inference"):
        time.sleep(0.02)  # Simulate model inference

    with FunctionProfiler("cv.postprocess"):
        time.sleep(0.003)  # Simulate post-processing

    return [{"bbox": [100, 100, 50, 50], "confidence": 0.8}]


class MockProcessor:
    @profile_method("process_frame", "pipeline")
    def process_frame(self, frame):
        """Mock frame processing pipeline"""
        process_image(frame)
        detections = detect_objects(frame)
        return detections


def main():
    print("🔧 Testing Profiling Decorators")
    print("=" * 40)

    # Create mock data
    mock_image = np.zeros((480, 640, 3), dtype=np.uint8)
    processor = MockProcessor()

    # Run multiple iterations to collect data
    print("Running profiling test iterations...")
    for i in range(20):
        processor.process_frame(mock_image)
        if i % 5 == 0:
            print(f"Completed {i+1}/20 iterations")

    # Get and display results
    stats = ProfileManager.get_instance().get_stats()
    print(f"\n📈 Profiling Results:")
    print(f"Node: {stats['node_name']}")
    print(f"Functions profiled: {len(stats['functions'])}")
    print()

    # Sort functions by total time
    sorted_functions = sorted(
        stats["functions"].items(), key=lambda x: x[1]["total_time"], reverse=True
    )

    print("Function Performance (sorted by total time):")
    print("-" * 60)
    print(f"{'Function':<25} {'Calls':<8} {'Avg (ms)':<10} {'Total (ms)':<12}")
    print("-" * 60)

    for func_name, func_stats in sorted_functions:
        avg_time = func_stats["avg_time"] * 1000
        total_time = func_stats["total_time"] * 1000
        call_count = func_stats["call_count"]
        print(f"{func_name:<25} {call_count:<8} {avg_time:<10.2f} {total_time:<12.2f}")

    # Export data
    export_filename = "profiling_test_results.json"
    export_profiling_data(export_filename)
    print(f"\n✅ Detailed results exported to {export_filename}")

    # Show top slowest functions
    print(f"\n🐌 Top 3 slowest functions (by average time):")
    top_slow = ProfileManager.get_instance().get_top_functions("avg_time", 3)
    for i, (func_name, avg_time) in enumerate(top_slow, 1):
        print(f"  {i}. {func_name}: {avg_time*1000:.2f}ms avg")


if __name__ == "__main__":
    main()
