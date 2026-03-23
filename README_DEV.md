# Developer Workflow for G1 Simulator

This guide explains the step-by-step process to deploy your code to the VM, build the Docker container, connect to VNC, and run the simulator using the provided helper scripts.

### 1. Deploy the Code (`deploy.sh`)
**Run this from your local machine.**
This script uses `rsync` to efficiently copy your local codebase to the `~/inorbit_g1_sim` folder on the remote VM. After copying, it automatically connects you to the VM via SSH and sets up port forwarding (`5901:localhost:5901`) for visualization.
*(Note: You can also use `connect.sh` if you just want to SSH into the VM without syncing files.)*

### 2. Connect to VNC (`connect_vnc.sh`)
**Run this from your local machine.**
This script opens the VNC viewer using the forwarded port established in step 1. This allows you to view the VM's desktop graphically. 
*(Note: You must keep the SSH connection from step 1 open for the port forwarding to work.)*

```bash
./connect_vnc.sh
```

### 3. Build the Docker Image (`build.sh`)
**Run this on the remote VM.**
This script builds the `inorbit_g1_sim` Docker image using the provided `Dockerfile`. It copies your local codebase directly into the image and explicitly clones the required `teleimager` submodule dependency.

```bash
./build.sh
```

### 4. Run the Docker Container (`run_docker.sh`)
**Run this on the remote VM after the build is complete.**
This script starts the container. It automatically sets up GPU access, network settings, and mounts specific files and folders (`sim_main.py`, `dds`, `tasks`) as volumes. Because these are mounted, any further code edits you make to those specific files on the VM host are immediately reflected inside the running container—no rebuild required!

```bash
./run_docker.sh
```

### 5. Run the Simulation (`run_simulation.sh`)
**Run this inside the bash shell of the running Docker container.**
Once the container starts and drops you into the shell, execute this script to actually start the Isaac Sim environment with the G1 robot in headless mode with Livestream 2 enabled, and activates the required DDS publishers.

```bash
./run_simulation.sh
```

### 6. View the Livestream via WebRTC (`run_webrtc.sh`)
**Run this inside your VNC session (on the VM GUI).**
Because the simulator runs in headless mode, you need to open the WebRTC client to actually see the simulation. Use the VNC desktop viewer (opened in step 2) to navigate to the VM and run the client script:

```bash
./run_webrtc.sh
```
