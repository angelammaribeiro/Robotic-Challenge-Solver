#!/bin/bash
# Setup script for agent_109061_102594_c3
# This agent is written in Python and uses the Webots controller API
# No additional dependencies or virtual environment required

# Exit on error
set -e

echo "[SETUP] Agent agent_109061_102594_c3 setup starting..."

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python3 is not installed or not in PATH"
    exit 1
fi

echo "[SETUP] Python3 found: $(python3 --version)"

# Verify required Python modules exist in the controller
if [ ! -f "gridNode.py" ]; then
    echo "[ERROR] gridNode.py not found"
    exit 1
fi

if [ ! -f "checkpoint.py" ]; then
    echo "[ERROR] checkpoint.py not found"
    exit 1
fi

echo "[SETUP] Required modules found"

# Clear Python cache to avoid import issues
if [ -d "__pycache__" ]; then
    echo "[SETUP] Clearing Python cache..."
    rm -rf __pycache__
fi

echo "[SETUP] Agent agent_109061_102594_c3 setup complete!"
exit 0
