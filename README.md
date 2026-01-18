# ROS2 Jenkins CI Waypoints

[![ROS2](https://img.shields.io/badge/ROS-2-blue)](https://docs.ros.org/en/humble/)
[![Jenkins](https://img.shields.io/badge/Jenkins-CI/CD-orange)](https://www.jenkins.io/)
[![Docker](https://img.shields.io/badge/Docker-Builds-blue)](https://www.docker.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A ROS2 project demonstrating CI/CD with Jenkins for the FastBot waypoints navigation package.

## Project Structure

```
├── src/                    # ROS packages
│   ├── fastbot_description/
│   ├── fastbot_gazebo/
│   └── fastbot_waypoints/
├── Jenkinsfile             # CI pipeline definition
└── Dockerfile              # Build environment
```

## CI/CD Setup

For Jenkins setup and quick access instructions, see the [jenkins-infra](https://github.com/legalaspro/ros1_jenkins_ci_waypoints/tree/main/jenkins-infra) in the ROS1 project. The same infrastructure works for both ROS1 and ROS2 pipelines.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
