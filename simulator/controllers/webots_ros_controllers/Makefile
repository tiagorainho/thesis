SHELL := /bin/bash

package=webots_ros2
launch=robot_launch.py

all: clean build run

clean:
	rm -rf ./build
	rm -rf ./install
	rm -rf ./log

build:
	colcon build --symlink-install --packages-select ${package}

run:
	source install/local_setup.bash
	ros2 launch webots_ros2 robot_launch.py