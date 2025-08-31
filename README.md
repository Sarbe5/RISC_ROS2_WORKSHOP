# RISC_ROS2_WORKSHOP

Welcome to the ROS2 Bootcamp repository! This repository contains practical examples and exercises to help you learn ROS2 (Robot Operating System 2) fundamentals, including publisher-subscriber patterns, turtle movement control, and package creation.

## 📋 Table of Contents

- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation & Setup](#installation--setup)
- [Package Overview](#package-overview)
- [Creating Your Own ROS2 Package](#creating-your-own-ros2-package)
- [Running the Examples](#running-the-examples)
- [Common Commands](#common-commands)
- [Troubleshooting](#troubleshooting)
- [Learning Resources](#learning-resources)

## 📁 Repository Structure

```
RISC_BOOTCAMP_ROS/
├── pack/                           # Basic ROS2 scripts collection
│   ├── __init__.py
│   ├── listner.py                  # Subscriber example
│   ├── move_circle.py              # Turtle circular movement
│   ├── move_spiral.py              # Turtle spiral movement
│   ├── move_square.py              # Turtle square movement
│   ├── move_st.py                  # Turtle straight line movement
│   ├── sf.py                       # Additional functionality
│   └── talker.py                   # Publisher example
├── publisher_node/                 # Complete ROS2 package example
│   ├── package.xml                 # Package metadata
│   ├── setup.py                    # Python package setup
│   ├── setup.cfg                   # Setup configuration
│   ├── publisher_node/             # Python module
│   │   ├── __init__.py
│   │   └── publisher_hello_node.py # Publisher node implementation
│   ├── resource/                   # Package resources
│   └── test/                       # Unit tests
├── pub_sub/                        # Publisher-Subscriber package
│   ├── package.xml
│   ├── setup.py
│   ├── pub_sub/
│   │   ├── __init__.py
│   │   ├── pub.py                  # Publisher node
│   │   └── sub.py                  # Subscriber node
│   ├── resource/
│   └── test/
└── README.md
```

## 🔧 Prerequisites

Before you begin, ensure you have the following installed:

- **Ubuntu 22.04** (recommended) or compatible Linux distribution
- **ROS2** (Humble/Iron/Rolling) - [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Python 3.8+**
- **Git**

### Verify ROS2 Installation

```bash
# Check if ROS2 is properly installed
ros2 --version

# Source ROS2 environment (add this to your ~/.bashrc)
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS2 version
```

## 🚀 Installation & Setup

### Prerequisites: Docker Environment Setup

This repository is designed to work with the franka_ros2 Docker environment. Make sure you have already set up the Docker workspace:

1. **Clone the franka_ros2 repository:**
   ```bash
   git clone https://github.com/Sarbe5/franka_ros2.git
   cd franka_ros2
   ```

2. **Set up Docker environment:**
   ```bash
   # Save current user ID
   echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
   
   # Build the container
   docker compose build
   
   # Run the container
   docker compose up -d
   
   # Open shell inside container
   docker exec -it franka_ros2 /bin/bash
   ```

### Setting Up This Learning Repository

**Inside the Docker container:**

1. **Navigate to the workspace source directory:**
   ```bash
   cd ~/ros2_ws/src
   ```

2. **Clone this learning repository:**
   ```bash
   git clone https://github.com/Sarbe5/RISC_ROS2_WORKSHOP
   cd RISC_BOOTCAMP_ROS
   ```

3. **Copy packages to workspace:**
   ```bash
   # Copy the complete packages to your workspace src directory
   cp -r publisher_node ~/ros2_ws/src/
   cp -r pub_sub ~/ros2_ws/src/
   ```

4. **Build the packages:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select publisher_node pub_sub
   source install/setup.bash
   ```

## 📦 Package Overview

### 1. `pack/` - Basic Scripts
Collection of standalone Python scripts demonstrating:
- **Publisher-Subscriber communication** (`talker.py`, `listner.py`)
- **Turtle movement patterns** (circle, spiral, square, straight line)

### 2. `publisher_node/` - Complete ROS2 Package
A properly structured ROS2 Python package showing:
- Package creation best practices
- Publisher node implementation
- Proper package.xml and setup.py configuration

### 3. `pub_sub/` - Publisher-Subscriber Package
Demonstrates the fundamental ROS2 communication pattern:
- **Publisher node** (`pub.py`) - Sends messages
- **Subscriber node** (`sub.py`) - Receives messages

## 🏗️ Creating Your Own ROS2 Package

### Step 1: Navigate to Your Workspace (Inside Docker Container)
```bash
cd ~/ros2_ws/src
```

### Step 2: Create a New Python Package
```bash
# Basic package creation
ros2 pkg create --build-type ament_python <package_name>

# Create package with dependencies
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs geometry_msgs
```

### Step 3: Package Structure
Your new package will have this structure:
```
your_package/
├── package.xml          # Package metadata and dependencies
├── setup.py            # Python package configuration
├── setup.cfg           # Setup configuration
├── your_package/       # Python module directory
│   └── __init__.py
├── resource/           # Package marker files
│   └── your_package
└── test/              # Unit tests directory
```

### Step 4: Edit setup.py
```python
from setuptools import setup

package_name = 'your_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_name = your_package.script_name:main',
        ],
    },
)
```

### Step 5: Create Your Node
Create a Python file in `your_package/your_node.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class YourNode(Node):
    def __init__(self):
        super().__init__('your_node')
        # Your node logic here

def main(args=None):
    rclpy.init(args=args)
    node = YourNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 6: Make It Executable
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select your_package

# Source the workspace
source install/setup.bash

# Make the script executable (if running directly)
chmod +x src/your_package/your_package/your_node.py
```

## 🏃‍♂️ Running the Examples

### Running Basic Scripts (pack/ directory)

**All commands should be run inside the Docker container:**

```bash
# First, open the Docker container shell
docker exec -it franka_ros2 /bin/bash

# Start turtlesim (required for turtle movement examples)
ros2 run turtlesim turtlesim_node

# In another terminal/container session, run movement scripts
docker exec -it franka_ros2 /bin/bash
cd ~/ros2_ws/src/RISC_BOOTCAMP_ROS/pack/
python3 move_circle.py
python3 move_square.py
python3 move_spiral.py

# For publisher-subscriber examples
python3 talker.py     # In one terminal
python3 listner.py    # In another terminal
```

### Running Package Nodes

**Inside the Docker container:**

```bash
# Source your workspace first
source ~/ros2_ws/install/setup.bash

# Run publisher node
ros2 run publisher_node publisher_hello_node

# Run pub_sub package
ros2 run pub_sub pub    # Publisher
ros2 run pub_sub sub    # Subscriber (in another container session)
```

## 🛠️ Common Commands

**All commands should be run inside the Docker container:**

```bash
# Access the container
docker exec -it franka_ros2 /bin/bash

# List all available packages
ros2 pkg list

# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name

# Show topic information
ros2 topic info /topic_name

# Build specific package
cd ~/ros2_ws
colcon build --packages-select package_name

# Build with verbose output
colcon build --event-handlers console_direct+

# Clean build
rm -rf build/ install/ log/
colcon build
```

## 🐛 Troubleshooting

### Common Issues and Solutions

**Working with Docker Environment:**

1. **"Package not found" error:**
   ```bash
   # Make sure you're inside the Docker container
   docker exec -it franka_ros2 /bin/bash
   
   # Source your workspace
   source ~/ros2_ws/install/setup.bash
   ```

2. **Import errors:**
   ```bash
   # Check if the package is properly built (inside container)
   cd ~/ros2_ws
   colcon build --packages-select your_package
   ```

3. **Permission denied:**
   ```bash
   # Make scripts executable (inside container)
   chmod +x your_script.py
   ```

4. **Turtle not moving:**
   ```bash
   # Make sure turtlesim is running (inside container)
   ros2 run turtlesim turtlesim_node
   ```

5. **Need multiple terminals:**
   ```bash
   # Open additional container sessions as needed
   docker exec -it franka_ros2 /bin/bash
   ```

### Environment Setup

**Inside the Docker container**, the ROS2 environment should already be configured. However, you may need to source your workspace:

```bash
# Add this to your container's ~/.bashrc if needed
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**For Development Outside Container:**
If you want to work outside the Docker container, add these lines to your `~/.bashrc`:
```bash
# ROS2 Environment Setup
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS2 version
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0  # Optional: set domain ID
```

## 📚 Learning Resources

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [Python Client Library (rclpy)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)


## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

**Happy Learning! 🚀**

*This repository is designed to help you get started with ROS2. Practice with the examples, experiment with the code, and don't hesitate to ask questions*
