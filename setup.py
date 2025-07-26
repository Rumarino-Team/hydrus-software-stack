#!/usr/bin/env python3
"""
Setup script for Hydrus Software Stack
"""

import os

from setuptools import find_packages, setup


# Read the README file
def read_readme():
    with open("README.md", "r", encoding="utf-8") as fh:
        return fh.read()


# Core dependencies
CORE_REQUIREMENTS = [
    "opencv-python",
    "numpy",
    "matplotlib",
    "requests",
    "colorama",
    "pyserial",
    "fastapi",
    "Flask",
    "black>=23.0.0",
    "isort>=5.12.0",
    "flake8>=6.0.0",
    "pre-commit>=3.0.0",
    "mypy>=1.0.0",
]

# Optional dependencies grouped by feature
OPTIONAL_REQUIREMENTS = {
    "depth-estimation": [
        "onnx",
        "onnxruntime-gpu",
        "onnxscript",
        "onnxslim",
        "torch",
        "torchvision",
        "tqdm",
        "typer",
    ],
    "ultralytics-acceleration": [
        "ultralytics",
        "onnx>=1.15.0",
        "onnxruntime>=1.16.0",
        "onnxruntime-gpu>=1.16.0",
        "torch>=1.8.0",
        "torchvision>=0.9.0",
    ],
    "tensorrt": [
        "tensorrt>=8.4.0",
        "pycuda>=2022.1",
        "nvidia-tensorrt",
        "nvidia-cudnn-cu12",
    ],
}

# Add 'all' extra that includes everything
OPTIONAL_REQUIREMENTS["all"] = [
    dep for deps in OPTIONAL_REQUIREMENTS.values() for dep in deps
]

setup(
    name="hydrus-software-stack",
    version="0.1.0",
    author="Cesar",
    author_email="cesar@example.com",
    description="Autonomous underwater vehicle software stack with computer vision and navigation capabilities",
    long_description=read_readme(),
    long_description_content_type="text/markdown",
    url="https://github.com/your-username/hydrus-software-stack",
    project_urls={
        "Bug Reports": "https://github.com/your-username/hydrus-software-stack/issues",
        "Source": "https://github.com/your-username/hydrus-software-stack",
    },
    packages=find_packages(include=["autonomy*"]),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
    install_requires=CORE_REQUIREMENTS,
    extras_require=OPTIONAL_REQUIREMENTS,
    entry_points={
        "console_scripts": [
            "hydrus-depth-estimator=autonomy.src.computer_vision.depth_estimation:main",
        ],
    },
    include_package_data=True,
    zip_safe=False,
)
