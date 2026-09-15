# Repository Guidelines

## Project Structure & Module Organization

This repository contains four ROS 2 `ament_cmake` packages. `camera_models/` provides calibration tools and reusable camera-model headers; `vins/` contains the estimator, feature tracker, factors, initialization code, ROS node, and launch files; `loop_fusion/` implements pose-graph loop closure; and `global_fusion/` combines VIO with GPS using its vendored GeographicLib subset. Runtime YAML, camera calibration, masks, and RViz settings live in `config/`. Images and BRIEF vocabulary data are under `support_files/`, while container helpers are in `docker/`.

## Build, Test, and Development Commands

Run ROS commands from the workspace root (two levels above this repository):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vins vins_rviz.launch.xml
```

Use `colcon build --packages-select vins` for a faster package-focused rebuild, adding dependent packages when needed. `colcon test && colcon test-result --verbose` runs registered ament checks. The repository currently has no dedicated unit-test suite; `kitti_odom_test` and `kitti_gps_test` are dataset-driven executables, not isolated unit tests. From `docker/`, `make build` builds the legacy container image.

## Coding Style & Naming Conventions

Code targets C++14 and is compiled with `-Wextra -Wpedantic` (plus `-Wall` in `global_fusion`). Follow the existing four-space indentation and brace-on-next-line style. Use `PascalCase` for classes, `camelCase` for methods and local variables, and descriptive snake_case for ROS parameters, topics, and YAML keys. Keep headers beside their owning module and source files under `src/`; avoid broad formatting-only changes because no repository-wide formatter configuration is present.

## Testing Guidelines

Every change should at least build all affected packages without new warnings. For estimator or fusion changes, replay the relevant EuRoC/KITTI/RealSense configuration and confirm node startup, topic publication, and RViz output. Add automated tests through a package's CMake/ament test block when practical, and name them after the behavior being verified.

## Commit & Pull Request Guidelines

Recent history favors short, imperative, lowercase subjects such as `add ros args support for global fusion node`; scoped prefixes such as `fix:` are also accepted. Keep each commit focused. Pull requests should explain the motivation, affected packages/configurations, build and runtime validation, and any parameter or topic compatibility impact. Link related issues and include screenshots or logs when visualization or trajectory behavior changes.
