# voxl-nodes

This repository generates an ipk package for ROS nodes for the VOXL platform, using ROS Melodic.
Note that the packages generated using the bionic-melodic docker as shown below will only work on a ROS-melodic docker on VOXL board.
To make packages runable directly on VOXL board, use the official emulator (ROS indigo, opencv2.4). 

## Procedure
For convenience, we create and edit a new node on the host machine that shares the catkin_ws repo with a VOXL emulator. After that, we can either share the catkin_ws/install repo on the host machine with the VOXL board or use the below instruction to generate an ipk package and then copy over, install on the VOXL board. 

We've already installed catkin_tools on the docker `arm64v8/melodic:bionic-melodic` that we'll instruct how to install in the next section.

## Build Emulator Instructions
There are official voxl-emulator docker images. However, they do not have the same packages that we want. Instead, we built our own docker, and then use emu to make the docker runable on a host machine (Ubuntu). Run the following command lines (on the host machine) to create a docker container that acts as a VOXL emulator.
```
git clone https://github.com/tynguyen/arm64v8_ubuntu_emulators
cd arm64v8_ubuntu_emulators
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
bash create_custom_docker_container.sh 
```
There might be some warning that can be ignored
```
WARNING: The requested image's platform (linux/arm64) does not match the detected host platform (linux/amd64) and no specific platform was requested
```

## Build Vox-nodes
Each node is a submodule, and some have their own own submodules too. Clone the main repo and submodules as follows. Best not to do this inside the docker image unless you want all files owned by root....

1) On the host machine:
```bash
~/git$ cd voxl-nodes/
~/git/voxl-nodes$ git submodule update --init --recursive
```

2) Launch the voxl emulator installed as aboveand build the project.
If using the official instruction to build the voxl-emulator, then,
```bash
~/git/voxl-nodes$ voxl-docker -i voxl-emulator
bash-4.3$ ./build.sh
```
In here, we will use our own way to run the voxl emulator as instructed above.
If the docker container has been created, run the following commands (on the host machine), 
```
docker start <docker_container_name>
docker exec -it <docker_cotainer_name> bash
```
By default, the docker container name would be `voxl_melodic_docker`.

3) Make an ipk package either inside or outside of docker.

```bash
bash-4.3$ exit
~/git/voxl-nodes$ ./make_package.sh

Package Name:  voxl-nodes
version Number:  0.0.1
ar: creating voxl-nodes_0.0.1_8x96.ipk

DONE
```

This will make a new voxl-nodes_x.x.x.ipk file in your working directory. The name and version number came from the ipk/control/control file. If you are updating the package version, edit it there.

## Install IPK on VOXL

You can now push the ipk package to the VOXL and install with opkg however you like. To do this over ADB, you may use the included helper script: install_on_voxl.sh. Do this outside of docker as your docker image probably doesn't have usb permissions for ADB.

Note that this only work with packages created inside the official emulator (ROS indigo). For ROS melodic packages, use other method.

```bash
~/git/voxl-vision-px4$ ./install_on_voxl.sh
pushing voxl-nodes_0.0.1_8x96.ipk to target
searching for ADB device
adb device found
voxl_vio_to_px4_0.0.1_8x96.ipk: 1 file pushed. 2.1 MB/s (51392 bytes in 0.023s)
Removing package voxl-nodes from root...
Installing voxl-nodes (0.0.1) on root.
Configuring voxl-nodes.

Done installing voxl-nodes
```

## Create a New Node
1) On the host machine, one can use catkin_tools to create a package
i.e
```
catkin create pkg <name_of_the_package> [independences]
```

2) Build the package
Inside the docker container (emulator), run
```
cd /root/emulator_ws
catkin build
catkin config --install
```
This will build and install the packges inside emulator_ws/src.

Now, we can either share the created install repo with the VOXL board or follow the above mentioned instructions to install the packages on the VOXL board

---
## Notes
When developing on the VOXL Emulator or on target, it may be necessary uninstall OpenCV 2.4 first. These are the commands to remove the old OpenCV 2.4:
```
opkg remove lib32-opencv --force-removal-of-dependent-packages
opkg remove lib32-libopencv --force-removal-of-dependent-packages
opkg remove lib32-libopencv-core2.4 --force-removal-of-dependent-packages
opkg remove lib32-opencv-dbg --force-removal-of-dependent-packages
```
