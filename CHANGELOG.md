# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [Unreleased]

### Added
- Added licenses.
- Added in `repos/rb_theron_sim.repos.yaml` the repository `gazebo_ros_pkgs` with the branch `noetic-devel`.
- Added docker generation cd pipelines for docker image building via github actions on 


### Changed
- Updated the version of the `package.xml` files to `0.1.1`.
- Modified in `repos/rb_theron_sim.repos.yaml` the branch of the repository `rb_theron_common` from `noetic-0.1.0` to `noetic-0.1.1-rc01`.
- Modified in `container/builder/Dokerfile` The nproc to use 60% on the cores on deb generation in order to save ram avoid machine crash (specially in gazebo_ros_pkgs).
- New docker compose structure with, include, override and merge features (consult docker compose documention for futher information). 
  - Definition of the services are made only once for the 3 docker compose run flavors.
  - Definition of the services are made only once for the local builder and ci.
  - Compose files with content are located in `container/compose`.
  - environment variables for the docker compose splitted and assigned to included.
  - Use yaml of anchors and links to avoid redefinition in the same file.
  - Refactor from `docker-compose.yaml` to `compose.yaml` to future compatibility with podman.
  - Removed `compose.env` and `.env` not required for use include feature of docker compose that allows to use several enviornment files.

### Fixed
- Fixed the wheels drift by updating `rb_theron_common` and added Robotnik's `gazebo_ros_control` install
- Changed the variable `ROS_DISTRO` to `DOCKER_ROS_DISTRO` to avoid possible interface with system environment.
- Refactor from `ci.yaml` (continuous integration) to `cd.yaml` (continuous delivery)

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