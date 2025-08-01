# Basic Instructions For URSim connection

1. Set up the URSim on Docker up until the ROS script startup. 

2. Setup the [URSim](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/ursim_docker.html) using Docker/Podman, following the instructions up until the ROS script. (Because the campus laptops run on Podman, the ROS2 startup script is not as reliable.) Instead, run the command below:
```bash
docker run --rm -it -p 5900:5900 -p 6080:6080 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --net ursim_net --ip 192.168.56.101 --name ursim docker.io/universalrobots/ursim_e-series 
```

3. In another terminal, run a ROS2 container that will be attached to for the ROS2 workspace.
```bash
docker run -it --network ursim_net --name ur_container -e DISPLAY=host.docker.internal:0.0 localhost/ur-ros2
```

Packages needed within the Docker container:
```
apt install ros-humble-ros2-control 
apt install ros-humble-ros2-controllers
apt install ros-humble-ur
apt install ros-humble-rmw_cyclonedds_cpp

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```