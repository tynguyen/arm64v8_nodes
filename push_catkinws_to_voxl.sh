# Push the install folder from catkin_ws to voxl board
rsync -azP catkin_ws/install voxl_board_home:/home/root/yocto_ws
