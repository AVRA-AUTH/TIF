# Third-party assets

`WAM-V-Base/`, `engine/`, and `propeller/` are the WAM-V catamaran mesh
models from the [VRX (Virtual RobotX)](https://github.com/osrf/vrx) project
(Open Source Robotics Foundation), licensed under Apache License 2.0 — see
`VRX_LICENSE` in this directory. Extracted from the `vrx_jazzy` Gazebo image;
unmodified other than being copied into this project's model search path.

The buoyancy/hydrodynamics plugins used in `worlds/exhibition_water.sdf`
(`libSurface.so` / `libSimpleHydrodynamics.so`, i.e. `vrx::Surface` /
`vrx::SimpleHydrodynamics`) are compiled binaries from that same VRX project
and license, copied into the Docker image from `vrx_jazzy` at build time
(see `docker/Dockerfile`) rather than checked into this repo.
