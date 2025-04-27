<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/gaultierlecaillon/edog_eurobot2025_ros2">
    <img src="https://i.postimg.cc/W1JzHDhC/eurobot-2024.jpg" alt="Logo">
  </a>

  <h3 align="center">Edog Eurobot 2025 ROS2 packages </h3>

  <p align="center">
    A collection of ROS2 packages to control the robot Edog 2025
    </p>
</div>


## Getting Started

Build the entire codebase
```shell
colcon build
```

Build a specific package
```shell
colcon build --packages-select actuator_package
```


Start the lidar (mandatory if motion)
```shell
lidar
```

Launch a strategy (startegy are located in the ./src/ia_package/resource folder)
```shell
launch strat:=test
```
