# oakd_isaac_ros

This is a package for interfacing the OAK-D PoE with the NVIDIA Isaac ROS Packages. 

## Launching nvBlox with the OAK-D PoE
## Step 1
Make sure you have a stable NVIDIA and CUDA environment. For x86_64 machine you will need Ubuntu 22.04+ and CUDA 12.6+. For Jetson devices you will need Jetpack 6.1 or Jetpack 6.2. 
Additionally, make sure you have the OAK-D connected, and give your machine a static ip such as ` 169.254.1.55 ` with netmask ` 255.255.0.0 `. Verfiy that the OAK-D is detected and data is being 
recieved by running ` python -m depthai_viewer `. 

If you can't see the OAK-D, follow this [troubleshooting guide.](https://docs.luxonis.com/hardware/platform/deploy/poe-deployment-guide#connected-to-the-same-lan-via-2-interfaces-wifi-ethernet)

## Step 2
Clone the repo and it's submodules (DepthAI ROS Driver, Isaac ROS Packages, ...) with the following command: 

```bash
mkdir ~/workspaces/ && cd ~/workspaces/ && \
git clone --recurse-submodules https://github.com/HJGrant/oakd_isaac_ros.git
```

## Step 3
Run the following script to build and launch the Docker container:

```bash
~/workspaces/oakd_isaac_ros/src/isaac_ros_common/scripts/run_dev.sh
```

## Step 4
Inside the container, set the ISAAC_ROS_WS variable and build the workspace:

```bash
export ISAAC_ROS_WS=/workspaces/oakd_isaac_ros/ && \
colcon build --symlink-install
```

## Step 5
After the build has completed, source the ROS2 environment with ` source install/setup.bash` and run the following command to run nvBlox with the OAK-D PoE:

```bash
ros2 launch my_oakd_launch nvblox_dynamics.launch.py
```

## Step 6
Visualize the output in Foxglove, by displaying the `/nvblox_node/color_layer` topic in a 3D Panel. 


## Building a Deployment Docker
If you would like to deploy this pipeline as a 1-click solution, that launches a Docker containter and automatically launches the correct ROS2 launch file, run the following command OUTSIDE of the docker container: 

```bash
~/workspaces/oakd_isaac_ros/src/isaac_ros_common/scripts/docker_deploy.sh --base_image_key "x86_64.ros2_humble.oakd" --ros_ws ~/workspaces/isaac_ros-dev --launch_package "my_oakd_launch" --launch_file "nvblox_dynamics.launch.py" -n "my_username/nvblox_dynamics_oakd" 
```

For Jetson Devices run (not tested):

```bash
~/workspaces/oakd_isaac_ros/src/isaac_ros_common/scripts/docker_deploy.sh --base_image_key "aarch64.ros2_humble.oakd" --ros_ws ~/workspaces/isaac_ros-dev --launch_package "my_oakd_launch" --launch_file "nvblox_dynamics.launch.py" -n "my_username/nvblox_dynamics_oakd_jetson_aarch64" 
```
Now you should see the image when you run `docker images` and you can then launch the pipeline by runnning: 

```bash
docker run --rm -it --gpus all --network host my_username/nvblox_dynamics_oakd 
```
