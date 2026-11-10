# Changelog

Format after [Keep a Changelog](https://keepachangelog.com/),
versioning after [SemVer](https://semver.org/).

## 2026-11-10 — The MoveIt viewer's config lives with the robot

- New package, from `deploy/offboard/a200/config/` in the workspace: `config/moveit.rviz` and `config/kinematics.yaml`,
  unchanged. `view-moveit` reads them through this package in `lite` on the robot and in the offboard viewer, as
  MARWIN 5's viewer reads `marwin_moveit_config`.
