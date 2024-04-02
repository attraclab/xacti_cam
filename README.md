# Xacti Gimbal Camera

This project is to control XACTI CX-GB400 gimbal and camera by using ROS topics.

## Install

- install libuvc by following steps
```sh

	## Install some dependencies
	sudo apt-get install -y libusb-1.0-0-dev libturbojpeg-dev

	## Install libucv
	cd /home/$USER
	git clone https://github.com/libuvc/libuvc.git
	cd /home/$USER/libuvc
	mkdir build
	cd build
	cmake ..
	make
	sudo make install
	sudo ldconfig

	## Setup udev rules
	echo 'SUBSYSTEM=="usb",  ENV{DEVTYPE}=="usb_device", GROUP="plugdev", MODE="0664"' | sudo tee /etc/udev/rules.d/10-libuvc.rules > /dev/null
	sudo udevadm trigger
	sudo usermod -a -G plugdev $USER

```

- [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation.html) 
- `sudo apt install python3-colcon-common-extensions`
- `sudo apt install ros-galactic-image-transport`
- install v4l2loopback by following steps

```sh
	## 1. Install v4l2loopback, choose either one of the following method
	### method 1
	sudo apt-get install -y v4l2loopback-utils
	### method 2
	cd /home/$USER/
	git clone https://github.com/umlaeute/v4l2loopback.git
	cd /home/$USER/v4l2loopback
	make && sudo make install
	sudo depmod -a


	## 2. Setup v4l2loopback to automaticall create /dev/video30 after boot
	sudo echo "v4l2loopback" >> /etc/modules-load.d/v4l2loopback.conf
	sudo echo "options v4l2loopback video_nr=30" >> /etc/modprobe.d/v4l2loopback.conf
	sudo echo "options v4l2loopback exclusive_caps=1" >> /etc/modprobe.d/v4l2loopback.conf
	sudo echo 'options v4l2loopback card_label="Fake Device"' >> /etc/modprobe.d/v4l2loopback.conf
	sudo update-initramfs -u
	reboot

```

- install this package by following steps

```sh
	## create workspace as dev_ws, if you don't have any workspace yet
	## or you can choose any name you want
	mkdir -p /home/$USER/dev_ws/src
	cd /home/$USER/dev_ws/src/
	git clone https://github.com/attraclab/xacti_cam.git
	source /opt/ros/galactic/setup.bash
	cd /home/$USER/dev_ws
	colcon build --symlink-install
	source install/local_setup.bash

```

## Topics Definition
| Topic name                  |       Msg type    | Definition            |
| :-------------------------- | :---------------: | :-------------------- |
| /xacti/camera/orientation   | std_msgs/msg/Int8 | define the camera orientation 0: lower side, 1: uppser side, 2: yaw is not activated (must be fixed mechanically), 3: auto judge |
| /xacti/camera/photo_capture | std_msgs/msg/Bool | either True or False then take a picture and save to SD card |
| /xacti/camera/focus_mode    | std_msgs/msg/Int8 | 0: MF (default), 1: S-AF, 2: C-AF |

## Run

Before running ROS2 node on any terminal, you will need to source ROS2 environment. You can run the following command one-by-one or you can put in inside `~/.bashrc`
```sh
export ROS_DOMAIN_ID=1
source /opt/ros/galactic/setup.bash
source ~/dev_ws/install/local_setup.bash
```


```sh
# Terminal 1
ros2 launch xacti_cam camera_launch.py

# Terminal 2
ros2 topic list # you should be able to see /xactic/** topics listing.


```

