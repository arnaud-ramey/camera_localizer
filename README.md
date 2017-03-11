# camera_localizer

[![Build Status](https://travis-ci.org/arnaud-ramey/camera_localizer.svg)](https://travis-ci.org/arnaud-ramey/camera_localizer)

Description
===========

"camera_localizer" is ...

Licence
=======

LGPL v3 (GNU Lesser General Public License version 3).
See LICENCE.

ROS driver node
===============

To launch the localizer:

```bash
$ roslaunch camera_localizer camera_localizer.launch
```

Node parameters
---------------

- `max_vel_lin  max_vel_ang`
[int, default: 100]

The maximum linear/angular velocity sent by the driver to the robot.
100 is the max speed.
Lower it to be kinder with the motors of the robot and
hence increase their life expectancy.

Subscriptions
-------------

- `cmd_vel`
[geometry_msgs::Twist, (m/s, rad/s)]

The instantaneous speed order.
Send it every 10 Hz to obtain continuous motion.

Publications
------------

- `camera/image_raw`
[sensor_msgs::Image]

The 640x480 raw image, encoded as `bgr8`.

Installation
============

You first need to install the official SDK (ARDrone3) by Parrot.
A summary of the instructions comes below.


How to install
==============

## 1. Dependencies from sources

Dependencies handling is based on the [wstool](http://wiki.ros.org/wstool) tool.
Run the following instructions:

```bash
$ sudo apt-get install python-wstool
$ roscd ; cd src
$ wstool init
$ wstool merge `rospack find camera_localizer`/dependencies.rosinstall
$ wstool update
```

## 2. Dependencies included in the Ubuntu packages

```bash
Ubuntu 14.04:
$ sudo apt-get install phablet-tools  autoconf  libavahi-client-dev  libavcodec-dev  libavformat-dev  libswscale-dev
Ubuntu 16.04:
$ sudo apt install repo  autoconf  libavahi-client-dev  libavcodec-dev  libavformat-dev  libswscale-dev
```

Build package with Catkin
-------------------------

```bash
$ catkin_make --only-pkg-with-deps camera_localizer
```
