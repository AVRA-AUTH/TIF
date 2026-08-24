# ASV Exhibition Simulator

A ROS 2 Humble + Gazebo simulation of an autonomous surface vessel (ASV), with a
browser-based control UI ([web_ui/](web_ui/)) for driving it, designing obstacle
courses, and running autonomous docking demos.

Everything runs inside a Docker container — you don't need ROS or Gazebo
installed on your machine, on any OS.

## Prerequisites

Install Docker for your platform:

- **Ubuntu / Linux**: [Docker Engine](https://docs.docker.com/engine/install/) (pick your distro from the list)
- **Windows**: [Docker Desktop](https://docs.docker.com/desktop/install/windows-install/), with the WSL2 backend (default on modern installs)
- **macOS**: [Docker Desktop](https://docs.docker.com/desktop/install/mac-install/)

No other software is required — the container carries its own Ubuntu 22.04 +
ROS 2 Humble + Gazebo environment.

## Quick start (all platforms)

Clone the repo (one-time):
```
git clone https://github.com/AVRA-AUTH/TIF.git
cd TIF/gazebo_deth
```

Then, every time you want to run it:
```
docker build -f docker/Dockerfile -t asv_exhibition .
docker run --rm -p 9090:9090 asv_exhibition
```
The first build downloads and compiles a fair amount (ROS 2 Humble desktop +
Gazebo + Nav2), so it can take several minutes. After that, Docker only
rebuilds layers that actually changed, so re-running the build command costs
almost nothing unless `docker/` or `src/` changed since last time. Once the
container logs settle (you'll see `Rosbridge WebSocket server started on port
9090`), it's ready.

Open [web_ui/index.html](web_ui/index.html) in a browser (just double-click
it, or drag it into a browser window). The status badge in the top-left
should turn green — **Connected to ROS**.

To stop the simulation, `Ctrl+C` in the terminal running `docker run` — the
`--rm` flag cleans up the container automatically.

## Platform-specific notes

### Ubuntu / generic Linux

The Quick Start above works as-is. On native Linux you can optionally use
`--network host` instead of `-p 9090:9090` — functionally equivalent here,
just one less flag to remember:

```
docker run --rm --network host asv_exhibition
```

If you're specifically on **Ubuntu 22.04** (not 24.04 or newer), you also
have the option to skip Docker entirely: install ROS 2 Humble + Gazebo
directly via [setup_ubuntu.sh](setup_ubuntu.sh), then from `gazebo_deth/`,
`colcon build`, `source install/setup.bash`, and
`ros2 launch asv_exhibition exhibition.launch.py`. (Note: [run.sh](run.sh) in
this folder assumes a WSL2 setup with a Windows-side source copy — it's not
meant for a true native Ubuntu install, so use the commands above instead.)
Docker is still recommended if you want your environment to match the rest
of the team exactly, regardless of your Ubuntu version.

### Windows

Use Docker Desktop with the WSL2 backend. Run the exact commands from the
Quick Start in PowerShell or Command Prompt — `-p 9090:9090` is required here
(`--network host` isn't reliably supported on Docker Desktop). Open
`web_ui\index.html` by double-clicking it in File Explorer; Docker Desktop
forwards the published port to Windows' `localhost` the same way it does on
Linux.

Expect the simulation to run noticeably slower than on native Linux — Docker
Desktop on Windows runs containers inside a VM, which adds overhead for a
physics-heavy sim like Gazebo.

### macOS

Use Docker Desktop the same way as Windows: `-p 9090:9090`, open
`web_ui/index.html` by double-clicking it.

**Apple Silicon (M1/M2/M3/M4) caveat:** the ROS 2 Humble base image this
project uses is only published for `amd64` (Intel/AMD), not `arm64`. On
Apple Silicon, Docker Desktop will run it under emulation (Rosetta), which
will be considerably slower — the build in particular may take a long time,
and simulation performance will suffer. Intel Macs run it natively with no
such penalty.

## What you'll see

The web UI has three modes:

- **Mode 1 — Level Designer & Auto Nav**: place buoys/obstacles and a goal,
  then run an automatic path-finding demo. This mode is entirely client-side
  JavaScript and works even without the container running (though then it's
  just a mockup, not real physics).
- **Mode 2 — Joystick Drive**: drive the boat with WASD/arrow keys, powered
  by real Gazebo physics over the ROS bridge. Requires the container running
  and the status badge to be green.
- **Mode 3 — Autonomous Docking**: click a berth on the zoomed-in marina map
  and watch the ASV navigate and dock itself.

Note: the 3D view in the web UI is its own hand-built Three.js model, not a
live render of Gazebo's simulation — only the boat's position/rotation are
synced from real odometry, its shape is fixed. To see Gazebo's actual
simulated model (e.g. the real WAM-V hull mesh), see GUI mode below.

## Development: editing the ROS code with working IntelliSense

If you're editing the ROS nodes ([src/asv_exhibition/scripts/](src/asv_exhibition/scripts/))
rather than just running the sim, plain VS Code won't have autocomplete or
jump-to-definition for `rclpy`, message types, etc. — those only exist inside
the container, not on your host. [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json)
sets up a VS Code Dev Container that attaches directly inside the same image
you already build above, where those paths are real:

1. Install the "Dev Containers" extension in VS Code.
2. Open the `gazebo_deth` folder itself in VS Code (not the repo root —
   this config is specific to this project).
3. Command palette → **Dev Containers: Reopen in Container**. First run
   builds the image (same one as above, so mostly cached if you've already
   built it); later runs are instant.

`src/` is live-mounted, so edits show up immediately without rebuilding the
image. To actually launch the sim from inside that container's terminal
(rather than a separate `docker run`):
```
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch asv_exhibition exhibition.launch.py headless:=true
```

## Advanced: GUI mode / physical joystick

By default the container runs Gazebo headless (no visible 3D window) and
only accepts keyboard/on-screen input through the web UI. Seeing Gazebo's
own native render window, or driving with a real USB gamepad instead of the
keyboard, needs extra setup (X11 socket passthrough for the former,
`/dev/input` device passthrough for the latter) that isn't wired up here.
Ask if you want that added.
