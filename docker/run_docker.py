#!/usr/bin/env python3
"""
Hydrus Docker Deployment - Main Entry Point
Simple wrapper that calls the modular hocker script
"""

import os
import sys
from pathlib import Path


def main():
    """Entry point that delegates to the hocker script"""
    hocker_script = Path(__file__).parent / "hydrus-docker" / "hocker"

    if not hocker_script.exists():
        print("❌ Error: hocker script not found in hydrus-docker folder")
        sys.exit(1)

    # Pass all arguments to the hocker script and replace current process
    try:
        # Use execve to replace current process with hocker
        env = os.environ.copy()
        os.execve(
            sys.executable, [sys.executable, str(hocker_script)] + sys.argv[1:], env
        )
    except KeyboardInterrupt:
        print("\n🛑 Operation cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error running hocker: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
