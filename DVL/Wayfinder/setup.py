import dvl
from setuptools import find_packages, setup

setup(
    name="dvl",
    version=dvl.__version__,
    author="trdi",
    author_email="rdifs@teledyne.com",
    url="http://www.teledynemarine.com/rdi/",
    packages=find_packages(),
    python_requires=">=3.6",
    install_requires=["pyserial", "numpy"],
)
