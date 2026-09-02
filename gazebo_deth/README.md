# ASV Exhibition Simulator

A ROS 2 Jazzy + Gazebo Harmonic simulation of an autonomous surface vessel (ASV),
with a browser-based control UI ([web_ui/](web_ui/)) for driving it, designing
obstacle courses, and running autonomous docking demos. Buoyancy and
hydrodynamic drag are real Gazebo physics (see `HANDOFF.md`), not faked.

Everything runs inside a Docker container — you don't need ROS or Gazebo
installed on your machine, on any OS.

## Prerequisites

Install Docker for your platform:

- **Ubuntu / Linux**: [Docker Engine](https://docs.docker.com/engine/install/) (pick your distro from the list)
- **Windows**: [Docker Desktop](https://docs.docker.com/desktop/install/windows-install/), with the WSL2 backend (default on modern installs)
- **macOS**: [Docker Desktop](https://docs.docker.com/desktop/install/mac-install/)

No other software is required — the container carries its own Ubuntu 24.04 +
ROS 2 Jazzy + Gazebo Harmonic environment. `docker/Dockerfile` builds VRX's
`libSurface.so`/`libSimpleHydrodynamics.so` buoyancy/hydrodynamics plugins
from source as its own build stage — no separate image or
pre-step needed; it's all part of the one `docker build` below. That stage
does compile ROS 2 + Gazebo + VRX from source, so the *first* build is slow
(tens of minutes, several GB) — normal, not a sign anything's wrong. Docker
caches it afterward, so it won't rerun unless that part of the Dockerfile
itself changes.

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
The first build downloads and compiles a fair amount (ROS 2 Jazzy desktop +
Gazebo + Nav2, plus VRX's buoyancy/hydrodynamics plugins built from source in
their own stage), so it can take several
minutes. After that, Docker only rebuilds layers that actually changed, so
re-running the build command costs almost nothing unless `docker/` or `src/`
changed since last time. Once the
container logs settle (you'll see `Rosbridge WebSocket server started on port
9090`), it's ready.

Open [web_ui/index.html](web_ui/index.html) in a browser (just double-click
it, or drag it into a browser window). The status badge in the top-left
should turn green — **Connected to ROS**.

To stop the simulation, `Ctrl+C` in the terminal running `docker run` — the
`--rm` flag cleans up the container automatically.

## Sharing it for an event

`web_ui/index.html` normally talks to rosbridge directly at `ws://localhost:9090`,
which only works when the page and the container are on the same machine. To
hand the demo to a phone/laptop over the internet, run the bundled server
instead of opening the file directly — it serves `web_ui/` and proxies the
rosbridge websocket under the same origin (`/rosbridge`), so one tunnel URL
covers both. `js/ros.js` already detects it's not on `localhost` and switches
to `wss://<that host>/rosbridge` automatically — no other setup needed.

This needs two extra things installed on the machine acting as the server
(not inside Docker, and not needed at all for plain local dev):
[Node.js](https://nodejs.org/) (any recent LTS) for `web_ui/server.js`, and
[`cloudflared`](https://github.com/cloudflare/cloudflared/releases/latest)
for the tunnel — grab the binary for your platform (e.g.
`cloudflared-linux-amd64`), `chmod +x` it, and put it on your `PATH`; no
account or `sudo` required for the quick-tunnel usage below.

Each time you want to run it, in three separate terminals (all three must
stay open/running for the whole session):

```
# 1. the simulation (build once per code change, run every time)
docker build -f docker/Dockerfile -t asv_exhibition .
docker run --rm -p 9090:9090 asv_exhibition
# wait for "Rosbridge WebSocket server started on port 9090" in its logs

# 2. the web server (serves web_ui/, proxies the websocket)
node web_ui/server.js 8080

# 3. the tunnel
cloudflared tunnel --url http://localhost:8080
```

`cloudflared` prints an `https://<random-words>.trycloudflare.com` URL near
the top of its output — that's what you send to testers. It's tied to that
one process: if the terminal running it closes or the process restarts,
you'll get a *new* URL and need to resend it. Quick tunnels also have no
uptime guarantee (Cloudflare's terms, not a bug) — fine for a demo, not for
anything that needs to stay up unattended.

To stop everything: `Ctrl+C` in each of the three terminals (Docker first is
safest, so nothing's left trying to reconnect to it).

Local dev (double-clicking `index.html`, or just `docker run -p 9090:9090`
without the server/tunnel) is unaffected by any of this.

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
source /opt/ros/jazzy/setup.bash
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
