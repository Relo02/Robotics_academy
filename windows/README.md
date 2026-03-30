# ROS2 Humble + Gazebo Harmonic - Windows Setup

## Prerequisites

1. **Docker Desktop for Windows**
   - Download from: https://www.docker.com/products/docker-desktop/
   - Enable **WSL2 backend** in Docker Desktop settings
   - Restart your computer after installation

2. **WSL2** (Windows Subsystem for Linux)
   ```powershell
   # Run in PowerShell as Administrator
   wsl --install
   wsl --set-default-version 2
   ```

3. **GUI Support** (choose one):

   ### Option A: WSLg (Windows 11 - Recommended)
   - Comes built-in with Windows 11
   - No additional setup needed
   - GUI applications work automatically

   ### Option B: VcXsrv (Windows 10)
   1. Download VcXsrv from: https://sourceforge.net/projects/vcxsrv/
   2. Install and launch **XLaunch**
   3. Configuration:
      - Select "Multiple windows"
      - Display number: 0
      - Select "Start no client"
      - Check "Disable access control" ⚠️
      - Save configuration for easy restart

## Quick Start

### Option 1: Using the start script (Recommended)

**PowerShell:**
```powershell
cd windows
.\start.ps1
```

**Command Prompt:**
```cmd
cd windows
start.bat
```

### Option 2: Manual commands
```powershell
# Create workspace
mkdir ros2_ws\src -Force

# Build the image (first time only, takes 10-20 min)
docker compose build

# Start the container
docker compose up -d

# Enter the container
docker compose exec -it ros2-gazebo bash
```

## Windows 10 Users (VcXsrv)

If you're using Windows 10 with VcXsrv:

1. Launch VcXsrv (XLaunch) with settings above
2. Edit `docker-compose.yml`:
   - Comment out the WSLg environment variables
   - Uncomment the VcXsrv line: `DISPLAY=host.docker.internal:0.0`
3. Rebuild: `docker-compose build`

## Testing the Installation

Inside the container:
```bash
# Test ROS2
ros2 topic list

# Test GUI (should open a window)
ros2 run rviz2 rviz2

# Test Gazebo
gz sim shapes.sdf
```

## Helpful Aliases

Inside the container:
- `cb` - Build workspace
- `sw` - Source workspace  
- `cbsw` - Build and source combined

## Stopping the Container

**PowerShell:**
```powershell
.\stop.ps1
```

**Or manually:**
```powershell
docker-compose down
```

## Troubleshooting

### "Cannot connect to Docker daemon"
- Make sure Docker Desktop is running
- Check Docker Desktop → Settings → General → "Use WSL 2 based engine" is enabled

### GUI not showing (WSLg)
```powershell
# Check WSLg is working
wsl --update
wsl --shutdown
# Then restart Docker Desktop
```

### GUI not showing (VcXsrv)
1. Make sure VcXsrv (XLaunch) is running
2. Check Windows Firewall isn't blocking VcXsrv
3. Verify "Disable access control" is checked in XLaunch

### Performance is slow
- This is normal with software rendering
- Close other applications to free up resources
- Consider using a Linux dual-boot for better performance

### Permission errors on workspace files
Inside the container:
```bash
sudo chown -R ros2user:ros2user ~/ros2_ws
```

### ROS2 nodes can't communicate between containers
- Use the same `ROS_DOMAIN_ID` for all containers
- Check Docker network settings

## File Locations

- Your ROS2 packages: `.\ros2_ws\src\`
- This folder is shared between Windows and the container
- Edit files with VS Code on Windows, run them in the container
