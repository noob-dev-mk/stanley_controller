# Stanley Controller on F1TENTH gym environment ROS2
Firstly, we will set the simulation environment, then implement the Stanley Controller. I will be using Docker with a NVIDIA GPU.

## Setup with an NVIDIA GPU:
- **Docker:** Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker. If you followed the post-installation steps, you won't have to prepend your Docker and docker-compose commands with sudo.
- **nvidia-docker2**, follow the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) if you have a support GPU. It is also possible to use Intel integrated graphics to forward the display, see the details instructions from the Rocker repo. If you are on Windows with an NVIDIA GPU, you'll have to use WSL (Windows Subsystem for Linux). Please refer to the guide [here](https://developer.nvidia.com/cuda/wsl), [here](https://docs.nvidia.com/cuda/wsl-user-guide/index.html), and [here](https://dilililabs.com/zh/blog/2021/01/26/deploying-docker-with-gpu-support-on-windows-subsystem-for-linux/).
- **rocker** https://github.com/osrf/rocker. This is a tool developed by OSRF to run Docker images with local support injected. We use it for GUI forwarding. If you're on Windows, WSL should also support this.
```bash
mkdir ros_ws
cd ros_ws/
mkdir stanley_ws
cd stanley_ws/
mkdir src 
cd src/
git clone https://github.com/f1tenth/f1tenth_gym_ros
cd f1tenth_gym_ros
docker build -t f1tenth_gym_ros -f Dockerfile .
```
**NOTE:** I like to keep all ROS-related work in ros_ws, you can just directly do it with stanley_ws and skip this.
Now the Docker setup is done, and we will use Rocker to mount and use the GPU.
To launch a container:
```bash
cd ros2_ws/stanley_ws
rocker --user --nvidia --x11 --volume "$(pwd)"/src:/sim_ws/src -- f1tenth_gym_ros
```
Sourcing and launching the simulation:
```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
The [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) (which we cloned earlier) is used to get the simulation environment. Do check the repository to know more about the simulation, as well as keyboard teleop. It only supports ROS 2 Foxy, hence we are using Docker. 

## Stanley Implementation
A separate stanley_pkg is created in the stanley_ws. The entire package is the repository with the same name.

Firstly, to get the Stanley Package (stanley_pkg), clone stanley_ws fron this repo into ros2_ws/stanley_ws/src/.
```bash
map_path: '/sim_ws/src/f1tenth_gym_ros/maps/<map_yaml_file_name>'
```
Now we need to change the map, add the map's .png and .yaml files in the map folder of the f1tenth_gym_gos package (Levine and Spielberg are already present). 

In the same package in the config file, go to sim.yaml and change:

```bash
cd ros2_ws/stanley_ws/src

```
We will use the Spielberg map for now as it is present by default, and its centerline.csv is also in the repository. So, **replace <map_yaml_file_name> with Spielberg_map**

**NOTE:** To get more maps and their raceline as well as centerline csv [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks.git).

To start the controller, first make sure the simulation is running. In a separate terminal, do the following:
```bash
docker exec -it <container_id> bash
ros2 run stanley_pkg stanley_final
```
(<container_id> can be found with: docker ps)

**NOTE:** The CSV values provided are in the opposite direction to the default sim heading direction, so change the sim.yaml of the f1tenth_gym_ros package stheta by pi to make it 180 degrees rotated by default.

```yaml
    # ego starting pose on map
    sx: 0.0
    sy: 0.0
    stheta: 3.14
```
After this, the car should follow the Spielberg map's centerline.
