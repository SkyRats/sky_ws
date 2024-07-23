# Install and setup

## Cloning this Repository

To clone this repository, and all of its submodules, use the following command:
```
git clone --recursive git@github.com:SkyRats/sky_ws.git
```

## 1. Ardupilot
Head to the Ardupilot fork submodule and install the required dependencies.

```
cd sky_ws/src/sky_base/ardupilot/
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
sudo pip3 install mavproxy
``` 
Then, set the ardupilot path on your .bashrc
```
echo "export PATH=$PATH:$HOME/sky_ws/src/sky_base/ardupilot/Tools/autotest" >> ~/.bashrc
echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.bashrc
source ~/.bashrc
```

Now that Ardupilot is installed, let's install ROS Noetic

## 2. ROS Noetic
Follow the instructions here: http://wiki.ros.org/noetic/Installation/Ubuntu

Remember to add the source command to your bashrc file:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you already have other ROS distros, be sure to remove their source commands from the bashrc file.

## 3. Gazebo 11

First, uninstall other versions of Gazebo:
```
sudo apt remove <other-gazebo-versions>
```

Follow the alternative installation: https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install

Add a line to end of `~/.bashrc` by running the following command:
```
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
```

## 4. Install Ardupilot Gazebo plugin

First, head to the the ardupilot_gazebo submodule, inside of the sky_base repository.
```
cd sky_ws/src/sky_base/gz_ws/src/ardupilot_gazebo
```
Then, build the plugin

```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

If you want to see the complete guide: https://ardupilot.org/dev/docs/sitl-with-gazebo-legacy.html#sitl-with-gazebo-legacy

## 5. Setup Catkin Workspace

Install the following catkin tools:

```
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```


Then, initialize the catkin workspace:

```
cd ~/sky_ws
mkdir build devel log
catkin init
```

## 6. Dependencies Installation
Install `mavros` and `mavlink` from source:
```
cd ~/sky_ws

sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

wstool init ~/sky_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```

Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/sky_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables:
```
source ~/.bashrc
```

Run the following command to tell Gazebo where to look for models:

```
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/sky_ws/src/sky_sim/models" >> ~/.bashrc
```
## 8. Build Instructions

Inside `sky_ws`, run `catkin build`:

```
cd ~/sky_ws
catkin build
```

Update global variables:
```
source ~/.bashrc
```

## 9. Usage

To test the simulation, you need at least two terminal windows:

- Run SITL:
```console
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
If the command is not found and the path can't be corrected, you might need to run the following command before running SITL:
```console
cd ~/sky_ws/src/sky_base/ardupilot/Tools/autotest
```

- Launch Gazebo and ROS:
```console
roslaunch sky_sim runway.launch
```

In addition, you can open QGround to test your flight simulation and RViz to visualize the camera image stream.

![Flight Test Demo](./docs/images/demo.png)

# Working with Submodules

Submodules are, simply put, repositories within repositories. In order to work with them, say, to commit, push, pull, etc. you can't just commit to the sky_ws repository, as all the changes you've made to the submodules won't be commited - instead, you have to either commit each submodule individually, or use the following command to do all the submodules at once.

```
git submodule foreach <your-desired-git-command>
```

then, do the same command to the sky_ws repository, as you would've normally.


## If you cloned this repository without the recursive flag

Just run the following commands to initialize the submodules

```
git submodule update --init --recursive
```

# Running sky_ws in a docker container

This session is for people who are not running Ubuntu 20.04 as their operation system. It is still **required** to clone the sky_ws repository. 
 
## Install docker
Follow the installation guide from docker documentation in respect to your operating system: [docker docs](https://docs.docker.com/engine/install/)

## Login into to docker ghcr.io
After downloading docker you will need to login. For this, you have to generate a classic token, so go to settings > developer settings > personal access tokens > Tokens (classic) and generate a token with write and delete scopes.

Next copy the token that appear in the screen and run the following command:
```zsh
sudo docker login --username <your_user> --password <token>
```

## Downloading a image
For downloading a docker imagame, just follow the command bellow:

```zsh
sudo docker pull ghcr.io/skyrats/sky_ws:latest
```

## Editing the run_image.bash
Using the text editor of your choice, open run_image.bash and change the following line: 
``` zsh
--volume = "<absolute/path/to/sky_ws>:/home/sky/sky_ws"
```

## Creating the container
Go to sky_ws directory, and just run the following command:
``` zsh
./run_image.bash
```

This may take a while if the sky_ws image is not installed. After complete, you will be prompt with a bash terminal inside the container.

## Executing a existing a container
You do not need to create a different container every time you want to run the sky_ws. After creating the container, you can enter it by using the following code:
``` zsh
./exec_image.bash
```

## Hints
 - just typing **Qground** will open the QgroundControl.AppImage
 - same password of sky computers

## Suggestions 
 - Install tmux: ``` sudo apt install tmux ```. Usefull if needed lots of terminal windowns





