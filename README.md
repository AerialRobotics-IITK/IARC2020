# IARC2020

ROS packages for solving [IARC Mission 9](http://aerialroboticscompetition.org/) simulation challenge.

## Setup

<details><summary>Create a catkin workspace if you don't have one:</summary>

```bash
mkdir -p ~/iarc_ws/src
cd ~/iarc_ws
catkin init
```

</details>

Clone this repository into your workspace:

```bash
cd ~/iarc_ws/src
git clone git@github.com:AerialRobotics-IITK/IARC2020
```

Update and fetch submodules:

```bash
cd IARC2020
git submodule update --init --recursive
```

Initialize wstool and install dependencies:

```bash
cd ~/iarc_ws/src
wstool init
wstool merge IARC2020/dependencies.rosinstall
wstool update
```

Other dependencies:

```bash
sudo apt-get install liblapacke-dev
```

Build the metapackage using:

```bash
catkin build iarc_mission_nine
```
