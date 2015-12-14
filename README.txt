# xm_object_cui

Create Date: 2015.11.1

Authors: frame_of_youth

Function: 
This package use pcl and opencv's libraries to recognize objects and it can store the result of identification in pdf file.

Dependance:
1. ROS use the version of opencv is 2.4.9 by default, but we need some functions about nonfree that is not included in opencv2.4.9, so we must download opencv2.4.11 from the opencv office website and compile and install manually. More details information, you can visit below website: http://docs.opencv.org/2.4.11/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation
2. Because of our match's requirement, we need screenhots when recognizing objects. So we use libharu library for generating PDF files. More detail information, you can visit below website: https://github.com/libharu/libharu/wiki/Installation

Use Guide:
1. roslaunch openni_launch openni_launch
2. rosrun xm_object_cui main_test


