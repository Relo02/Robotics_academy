# ROS2 Humble + Gazebo Harmonic Docker Setup

This Docker environment provides a complete ROS2 Humble and Gazebo Harmonic setup for the robotics academy course.

## Prerequisites

### All Platforms
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) installed and running

### Windows
1. Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)
2. Enable WSL2 backend in Docker Desktop settings
3. Install an X Server (choose one):
   - **Option A (Recommended)**: Use WSL2 with WSLg (Windows 11) - GUI works automatically
   - **Option B**: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [X410](https://x410.dev/)

**If using VcXsrv on Windows:**
```powershell
# Launch VcXsrv with these settings:
# - Multiple windows
# - Start no client
# - Check "Disable access control"
# Then set DISPLAY variable:
set DISPLAY=host.docker.internal:0
```

### Linux
```bash
# Allow X11 forwarding
xhost +local:docker
```

### macOS
1. Install [XQuartz](https://www.xquartz.org/)
2. Open XQuartz → Preferences → Security → Enable "Allow connections from network clients"
3. Restart XQuartz
```bash
xhost +localhost
export DISPLAY=host.docker.internal:0
```

## Quick Start

### 1. Clone/Download this folder
Make sure you have all files:
- `Dockerfile`
- `docker-compose.yml`
- `README.md` (this file)

### 2. Build the Docker image
```bash
docker-compose build
```
*This may take 10-20 minutes on first run.*

### 3. Start the container
```bash
docker-compose up -d
```

### 4. Enter the container
```bash
docker exec -it ros2_course_container bash
```

### 5. Test the installation
Inside the container:
```bash
# Test ROS2
ros2 topic list

# Test Gazebo Harmonic
gz sim shapes.sdf
```

## Usage Guide

### Starting/Stopping the Container
```bash
# Start
docker-compose up -d

# Stop
docker-compose down

# Restart
docker-compose restart
```

### Working with ROS2
Inside the container, your workspace is at `~/ros2_ws`. Use these helpful aliases:

| Alias | Command | Description |
|-------|---------|-------------|
| `cb` | `colcon build --symlink-install` | Build the workspace |
| `sw` | `source install/setup.bash` | Source the workspace |
| `cbsw` | Build + Source | Combined build and source |

### Creating a New Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_package
# or for Python:
ros2 pkg create --build-type ament_python my_python_package
```

### Running Gazebo with ROS2
```bash
# Launch empty world
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"

# With a robot (example)
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r shapes.sdf"
```

### RViz2
```bash
rviz2
```

## Persistent Storage

Your code is stored in the `./ros2_ws` folder on your host machine. This means:
- Your code persists even if you delete the container
- You can edit code with your favorite IDE on your host machine
- Changes are immediately visible inside the container

## Troubleshooting

### GUI Not Working

**Windows (WSL2):**
```powershell
# If using WSLg (Windows 11), it should work automatically
# If not, try setting:
$env:DISPLAY="host.docker.internal:0"
docker-compose up -d
```

**Linux:**
```bash
xhost +local:docker
# If still not working, check DISPLAY variable:
echo $DISPLAY
# Should show something like ":0" or ":1"
```

**macOS:**
```bash
# Make sure XQuartz is running
xhost +localhost
export DISPLAY=host.docker.internal:0
```

### Performance Issues
If simulation is slow, try enabling software rendering (already set in docker-compose.yml):
```yaml
environment:
  - LIBGL_ALWAYS_SOFTWARE=1
```

### "Permission denied" errors
```bash
# Fix ownership of workspace
sudo chown -R ros2user:ros2user ~/ros2_ws
```

### Container won't start
```bash
# Check logs
docker-compose logs

# Remove and rebuild
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

### Network Issues (ROS2 nodes can't communicate)
Make sure `network_mode: host` is set in docker-compose.yml, and all students use the same `ROS_DOMAIN_ID`.

## Useful Commands Reference

### ROS2 Commands
```bash
# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic_name

# List all nodes
ros2 node list

# Get node info
ros2 node info /node_name

# Run teleop keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Gazebo Commands
```bash
# Launch Gazebo with a world
gz sim world.sdf

# List Gazebo topics
gz topic -l

# Echo Gazebo topic
gz topic -e -t /topic_name
```

## Support

If you encounter issues:
1. Check the Troubleshooting section above
2. Search the [ROS2 Humble documentation](https://docs.ros.org/en/humble/)
3. Check [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic)
4. Ask your instructor or TAs

---
*ROS2 Course - Docker Environment v1.0*
