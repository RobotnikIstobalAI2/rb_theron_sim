# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [Unreleased]

## [noetic-0.1.0] - 2024-02-05

### Added
- Added multirobot support with the environment variable `ROBOT_QTY` can simulate up 5 robots.
- Added [pose_publisher](https://github.com/RobotnikAutomation/pose_publisher) repository to `repos`file.
- Added environment variable to `LAUNCH_POSE_PUBLISHER` to launch the node_pose_publisher. 
- Added environment variable to `POSE_PUBLISHER_FREQUENCY` to change default value of the publishing frequency for the node_pose_publisher.
- Added environment variable to `POSE_PUBLISHER_BASE_FRAME_SUFFIX` to change default value of robot base frame suffix.
- Added environment variable to `POSE_PUBLISHER_TOPIC_REPUB` to change default value of the topic name where the node_pose_publisher publishes.
- Added environment variable to `LAUNCH_WEB_THROTTLE` to launch the throttle publisher.
- Added environment variables in the container structure.
- Added environment file `.env` for  docker compose unified variables.
- Added environment file `compose-config.env` for docker compose.
- Added image-transport-plugins for compressed images in the container structure.
- Added web interface in the container structure (in the folder `web-cpu-only`), so no need of graphical user interface.
- Added conditional change of the ROS repositories.
- Added GitHub actions for automatic release creation.
- Added `doc` folder for documentation files.

### Changed
- Modified the `rb_theron_complete.launch` to add the pose_publisher `.launch` and read the environment variables to configure it.
- Modified the `rb_theron_gazebo.launch` to launch and configure the pose_publisher node.
- Modified the `rb_theron_robot.launch` to launch and configure the pose_publisher node.
- Modified the `rb_theron_sim.repos.yaml` to change the version of the `robotnik_msgs` branch to `ros` (instead of `master`).
- Updated the `rb_theron_sim.repos.yaml` to change the version of the `rb_theron_common` branch to `noetic-0.1.0`.
- Updated the `rb_theron_sim.repos.yaml` to change the version of the `rb_theron_sim` branch.
- Increased the version ira_laser_tools in order to force the installation of our packages.
- Updated the docker-compose `.yaml` (`nvidia` and `intel`) to add and configure the default values of the new environment variables.
- Moved the `docker-compose.yaml` of `intel` to `gui-cpu-only`.
- Moved the `docker-compose.yaml` of `nvidia` to `gui-gpu-nvidia`.
- Changed container structure to be more clear.
- Changed the path of the container builder.
- Updated the image base version to `0.5.0`.
- Use of `apt-fast` for parallel downloading for `apt`.
- Updated `README.md` file.
  - Update environment variables.
  - Updated docker usage.
  - Update docker environment variables.
- Updated `CHANGELOG.md` file.