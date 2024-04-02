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
| Topic name                      |       Msg type     | Definition            |
| :------------------------------ | :---------------:  | :-------------------- |
| /xacti/camera/orientation       | std_msgs/msg/Int8  | define the camera orientation, <br>0: lower side <br>1: uppser side <br>2: yaw is not activated (must be fixed mechanically)<br> 3: auto judge |
| /xacti/camera/photo_capture     | std_msgs/msg/Bool  | either True or False then take a picture and save to SD card |
| /xacti/camera/focus_mode        | std_msgs/msg/Int8  | Change focus mode, <br>0: MF (default) <br>1: S-AF <br>2: C-AF |
| /xacti/camera/focus_mm          | std_msgs/msg/Int32 | Focus position in mm from 300 - 100000(default) |
| /xacti/camera/video_record      | std_msgs/msg/Bool  | Start or stop recording video to SD card <br>True: start recording <br>False: stop recording |
| /xacti/camera/video_resolution  | std_msgs/msg/Int8  | Change the video resolution, <br>0: 4K (default) <br>1: 2.7K <br>2: Full HD <br>3: HD |
| /xacti/camera/optical_zoom      | std_msgs/msg/Int32 | Do optical zoom, data from 100 to 250 with incremented by 10 |
| /xacti/camera/iso               | std_msgs/msg/Int8  | Change ISO sensitivity <br>0 : auto <br>1 : 125 (default) <br>2 : 160 <br>3 : 200 <br>4 : 250 <br>5 : 320 <br>6 : 400 <br>7 : 500 <br>8 : 640 <br>9 : 800 <br>10 : 1000 <br>11 : 1250 <br>12 : 1600 <br>13 : 2000 <br>14 : 2500 <br>15 : 3200 <br>16 : 4000 <br>17 : 5000 <br>18 : 6400 |
| /xacti/camera/aperture          | std_msgs/msg/Int8  | Change aperture <br>0 : F2.8 (default) <br>1 : F3.2 <br>2 : F3.5 <br>3 : F4.0 <br>4 : F4.5 <br>5 : F5.0 <br>6 : F5.6 <br>7 : F6.3 <br>8 : F7.1 <br>9 : F8.0 <br>10 : F9.0 <br>11 : F10.0 <br>12 : F11.0 |
| /xacti/gimbal/restart           | std_msgs/msg/Bool  | Restart and do initialization on gimbal based on camera orientation steup |
| /xacti/gimbal/pan               | std_msgs/msg/Int16 | Control pan angle from -85 to 85 degrees |
| /xacti/gimbal/tilt              | std_msgs/msg/Int16 | Control tilt angle <br>tilt (lower mount): -115 deg to 45 deg <br>tilt (upper mount): -45 |deg to 115 deg |
| /xacti/camera/view              | sensor_msgs/msg/Image | Image message to view camera streaming |

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

