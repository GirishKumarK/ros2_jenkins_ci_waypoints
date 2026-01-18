# ROS2 Jenkins CI Waypoints

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
