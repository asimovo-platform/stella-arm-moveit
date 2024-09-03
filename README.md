# Asimovo ROS 2 Stella Arm Simulation Setup

This README provides instructions on how to set up and run the Asimovo ROS 2 stella simulation using Docker.

## Prerequisites

Ensure you have Docker installed on your machine. If not, follow the instructions [here](https://docs.docker.com/get-docker/) to install Docker.

## Step 1: Clone this repo in your local machine (which has 'asimovo-humble-fortress' image installed) in your workspace (Terminal 1)

```bash
mkdir <your_workspace_name>  && cd <your_workspace_name>
git clone https://github.com/ritwik-asimovo/stella-arm-moveit.git
```

## Step 2: Run the Docker Container (Terminal 1)

Run the Docker container with the necessary environment settings and volume mounts:

```bash
sudo docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --network="host" -it asimovo-humble-fortress:latest
```

## Step 3: Copy the installed packages in your docker container (Terminal 2)

Check container ID and copy the container ID:

```bash
sudo docker ps
```

Now Copy the packages into the docker container using:

```bash
sudo docker cp /home/<user>/<your_workspace_name>/stella-arm-moveit/. <container ID>:/home/asimovo
```

## Step 4: Now in Terminal 1 run the shell file to install all dependencies and build the packages (Terminal 1)


```bash
cd /home/asimovo
chmod +x run.sh
./run.sh
```

## Step 5: Source the workspace and launch the bringup (Terminal 1)


```bash
source install/setup.bash
ros2 launch bringup_stella_arm bringup.launch.py
```


To test moveit, in Rviz, select stella_arm as planning group and set start state to current and goal state to 'home'.
Now select stella_gripper as planning group and set start state as current and goal state to 'gripper_open'
