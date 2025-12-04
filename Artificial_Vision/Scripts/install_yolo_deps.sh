#!/bin/bash

echo "Installing minimal required dependencies for YOLOv8 and OpenCV..."
echo "-------------------------------------------------"

# Install NumPy (specific version for YOLOv8)
echo "INSTALLING NUMPY..."
#pip3 install numpy==1.23.5

# Install OpenCV (headless)
echo "-------------------------------------------------"
echo "INSTALLING OPENCV..."
pip3 install opencv-python  #headless

# Install PyTorch (CPU-only)
echo "-------------------------------------------------"
echo "INSTALLING TORCH (CPU-only)..."
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Install PyYAML
echo "-------------------------------------------------"
echo "INSTALLING PYYAML..."
pip3 install pyyaml

# Install Matplotlib
echo "-------------------------------------------------"
echo "INSTALLING MATPLOTLIB..."
pip3 install matplotlib

# Install Polars
echo "-------------------------------------------------"
echo "INSTALLING POLARS..."
pip3 install polars

# Install psutil
echo "-------------------------------------------------"
echo "INSTALLING PSUTIL..."
pip3 install psutil

# Install requests
echo "-------------------------------------------------"
echo "INSTALLING REQUESTS..."
pip3 install requests

# Install SciPy
echo "-------------------------------------------------"
echo "INSTALLING SCIPY..."
pip3 install scipy

# Install ultralytics-thop (required by ultralytics)
echo "-------------------------------------------------"
echo "INSTALLING ULTRALYTICS-THOP..."
pip3 install ultralytics-thop

# Install ultralytics (without optional dependencies)
echo "-------------------------------------------------"
echo "INSTALLING ULTRALYTICS (without optional dependencies)..."
pip3 install ultralytics --no-deps

# Install Flask (optional, for web interface)
echo "-------------------------------------------------"
echo "INSTALLING FLASK..."
pip3 install Flask

echo "-------------------------------------------------"
echo "Minimal dependencies installed successfully!"
echo "Ready to run YOLOv8!"
