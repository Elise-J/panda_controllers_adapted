## License
The code is from https://github.com/CentroEPiaggio/panda_controllers/tree/stefano-darko

and modifed to control the robot with ros 

## Docker launch

aica-docker interactive panda_controllers:noetic -u ros --net host --no-hostname -v /home/alberic/Documents/LASA/elise_save_file/panda_controllers_adapted/panda_controllers:/home/ros/ros_ws/src/panda_controllers --cap-add=sys_nice --ulimit rtprio=99

