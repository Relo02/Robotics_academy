@echo off
REM Start script for Windows - ROS2 Humble + Gazebo Harmonic
REM Double-click this file or run from Command Prompt

echo ==========================================
echo ROS2 Humble + Gazebo Harmonic - Windows
echo ==========================================

REM Create workspace directory if it doesn't exist
echo [1/3] Checking workspace directory...
if not exist ".\ros2_ws\src" mkdir ".\ros2_ws\src"

REM Build and start the container
echo [2/3] Starting Docker container...
docker-compose up -d

if %ERRORLEVEL% NEQ 0 (
    echo Failed to start container. Check Docker Desktop is running.
    pause
    exit /b 1
)

REM Enter the container
echo [3/3] Entering container...
echo.
echo You are now inside the ROS2 container.
echo Type 'exit' to leave the container.
echo ==========================================
echo.

docker exec -it ros2-gazebo bash
pause
