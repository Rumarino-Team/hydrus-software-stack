#!/usr/bin/env python3
"""
Hydrus Docker Deployment - Main Entry Point
Simple wrapper that calls the modular hocker script
"""

import sys
import subprocess
from pathlib import Path

def main():
    """Entry point that delegates to the hocker script"""
    hocker_script = Path(__file__).parent / "hydrus-docker" / "hocker"
    
    if not hocker_script.exists():
        print("❌ Error: hocker script not found in hydrus-docker folder")
        sys.exit(1)
    
    # Pass all arguments to the hocker script
    try:
        result = subprocess.run([sys.executable, str(hocker_script)] + sys.argv[1:], 
                              check=False)
        sys.exit(result.returncode)
    except KeyboardInterrupt:
        print("\n🛑 Operation cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error running hocker: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()