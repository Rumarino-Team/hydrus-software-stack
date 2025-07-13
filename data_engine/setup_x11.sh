#!/bin/bash

# Setup X11 authentication for Docker GUI applications
echo "Setting up X11 authentication for Docker..."

# Create X11 auth file if it doesn't exist
XAUTH_FILE="/tmp/.docker.xauth"

# Remove old auth file if it exists
if [ -f "$XAUTH_FILE" ]; then
    rm "$XAUTH_FILE"
fi

# Create new auth file
touch "$XAUTH_FILE"

# Add current display to auth file
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge -

# Set proper permissions
chmod 644 "$XAUTH_FILE"

echo "X11 authentication setup complete!"
echo "DISPLAY: $DISPLAY"
echo "XAUTH_FILE: $XAUTH_FILE"

# Allow X11 connections from localhost (for Docker)
xhost +local:docker

echo "You can now run: docker compose up"
