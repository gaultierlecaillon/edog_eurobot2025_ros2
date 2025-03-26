<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="https://i.postimg.cc/m2LKh4Fk/banner.png" alt="Logo">
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

Launch a strategy
```shell
launch strat:=test_strat
```
