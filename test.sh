#!/bin/bash
set -e  # Exit on error

# Build the project
cd build 
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
make 

# Generate Python stubs
cd python
PYTHONPATH=. pybind11-stubgen franka_py

# Move stubs to the correct location
mv stubs/franka_py.pyi .
rmdir stubs

# Create stubs package
echo "Creating and installing stubs package..."

# Create setup.py for the stubs package
cat > setup.py << 'EOL'
from setuptools import setup, find_packages

setup(
    name="franka_py",
    version="0.1",
    packages=find_packages(),
    py_modules=["franka_py"],
)
EOL

# Install the stubs package
pip install -e .
cd ../..

# Uncomment to run the test script
#python3 test.py

echo "Build and stubs generation completed successfully!"