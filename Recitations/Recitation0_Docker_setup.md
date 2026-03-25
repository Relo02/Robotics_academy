# ROS2 development setup

This is a guide to make the setup process easier for everyone, some may find the steps trivial but it's to get everybody on board before we start the development.

### Install Docker

[﻿https://www.docker.com/products/docker-desktop/](https://www.docker.com/products/docker-desktop/)

**Possible errors**

- you have to enable hardware assisted virtualization on windows [look here](https://forums.docker.com/t/hardware-assisted-virtualization-and-data-execution-protection-must-be-enabled-in-the-bios/109073)
- if you have an older version of operating system you may want to install an older version of docker [﻿look here](https://docs.docker.com/desktop/release-notes/)

### Install GitHub Desktop

[https://desktop.github.com/](https://desktop.github.com/)

GitHub Desktop can be used to clone the repository and manage commits with a graphical interface.

### Install VS Code

[https://code.visualstudio.com/](https://code.visualstudio.com/)

VS Code will be used to open the repository and edit the project files.

### Description

This Docker container is set up for running a robotic simulation environment with the following specifications:

**ROS Version:** The container is configured to run with ROS (Robot Operating System) in the "Humble" release.

**Gazebo:** Gazebo Ignition, a simulation tool

**Operating System:** The base operating system is Ubuntu 22.04.3 LTS.

---

# Docker Container

- **important note**: in docker before executing the following commands you have to set in preferences -> resources -> file sharing the path to the github folder

MacOS:

```
docker run -d \
  --pull always \
  -p 6080:80 \
  -p 9090:9090 \
  --security-opt seccomp=unconfined \
  --shm-size=512m \
  -v "/Users/<username>/<path_to_github>/yourrepo:/github" \
  --name racademy \
  voss01dev/racademy:arm64
```

Windows:
use regular shell CMD (do not use wsl)

```
docker run -d ^
  --pull always ^
  -p 6080:80 ^
  -p 9090:9090 ^
  --security-opt seccomp=unconfined ^
  --shm-size=512m ^
  -v "C:\Users\<username>\<path_to_github>:/github" ^
  --name racademy ^
  voss01dev/racademy:amd64
```

At this point you should get as output a bunch of `RUNNING state` lines and you can proceed.

### Note

The `-v /Users/<username>/<path_to_github>:/github` part of the Docker run command establishes a volume mount. This allows you to share data between your host machine and the Docker container. Here's a breakdown of this volume mount:

`/Users/<username>/<path_to_github>`: with the actual path on your host machine that corresponds to the github folder, that way it will be accessible from the Docker container.

`:/github`: This is the path inside the Docker container where the shared data will be available. In this case, it's mounted at `/github`.

### Accessing the GUI

After running the container, you can access the graphical user interface (GUI) by opening a web browser and navigating to `http://localhost:6080`. The container exposes the GUI on port 6080, allowing you to interact with the simulation environment.

### To restart the container when closed

(you should always have the docker app running in the background) in the computer terminal/shell write:
To start the stopped container:

```
docker start racademy
```

To enter into the shell:

```
docker exec -it --user ubuntu racademy bash
```

---
