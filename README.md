# voxl-nodes

This repository generates an ipk package for ROS nodes for the VOXL platform.


## Build Instructions

1) prerequisite: voxl-emulator docker image

Follow the instructions here:

https://gitlab.com/voxl-public/voxl-docker


2) Clone Submodules

Each node is a submodule, and some have their own own submodules too. Clone the main repo and submodules as follows. Best not to do this inside the docker image unless you want all files owned by root....

```bash
~/git$ cd voxl-nodes/
~/git/voxl-nodes$ git submodule update --init --recursive
```

3) Launch the voxl-emulator docker image and build the project.
If using the official instruction to build the voxl-emulator, then,
```bash
~/git/voxl-nodes$ voxl-docker -i voxl-emulator
bash-4.3$ ./build.sh
```
In here, we will use our own way to run the voxl-emulator (even though the image is the same), therefore, first clone 
[this repo](https://github.com/tynguyen/arm64v8_ubuntu_emulators) and then run 
```
cd arm64v8_ubuntu_emulators
bash create_official_docker_emulator_container.sh 
```
If the docker container has been created, run the following commands (on the host machine) instead, 
```
docker start <docker_container_name>
docker exec -it <docker_cotainer_name> bash
```

4) Make an ipk package either inside or outside of docker.

```bash
bash-4.3$ exit
~/git/voxl-nodes$ ./make_package.sh

Package Name:  voxl-nodes
version Number:  0.0.1
ar: creating voxl-nodes_0.0.1_8x96.ipk

DONE
```

This will make a new voxl-nodes_x.x.x.ipk file in your working directory. The name and version number came from the ipk/control/control file. If you are updating the package version, edit it there.

### Install IPK on VOXL

You can now push the ipk package to the VOXL and install with opkg however you like. To do this over ADB, you may use the included helper script: install_on_voxl.sh. Do this outside of docker as your docker image probably doesn't have usb permissions for ADB.


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

