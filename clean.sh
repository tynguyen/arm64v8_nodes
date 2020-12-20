#!/bin/bash
#
# cleans the ros workspace
#
# Modal AI Inc. 2019
# author: james@modalai.com



sudo rm -rf catkin_ws/build/
sudo rm -rf catkin_ws/install/
sudo rm -rf catkin_ws/devel/

rm -rf ipk/control.tar.gz
sudo rm -rf ipk/data/
rm -rf ipk/data.tar.gz
rm -rf *.ipk