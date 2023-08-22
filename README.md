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
``` 
Now that Ardupilot is installed, let's install ROS Noetic

## 2. ROS Noetic
Follow the instructions here: http://wiki.ros.org/noetic/Installation/Ubuntu

Remember to add the source command to your bashrc file:
```
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
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

First, head to the sky_base repository, and create the required folders for the install.

```
cd sky_ws/src/sky_base
mkdir -p gz_ws/src
```
Then, clone the plugin's repository in the folder you created, and build it.


```
cd gz_ws/src
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
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
catkin init
```

## 6. Dependencies Installation
Install `mavros` and `mavlink` from source:
```
cd ~/sky_ws
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

install geographiclib dependancy:
```
sudo ~/sky_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
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

- Launch Gazebo and ROS:
```console
roslaunch sky_sim runway.launch
```

In addition, you can open QGround to test your flight simulation and RViz to visualize the camera image stream.

![Flight Test Demo](./docs/images/demo.png)
