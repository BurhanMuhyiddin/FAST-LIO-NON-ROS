# Dillinger
## About

This repository contains modified FAST-LIO which doesn't depend on ROS. So, it can be easily run in both windows and Colab.

## How to build and run
```sh
$git clone https://github.com/BurhanMuhyiddin/FAST-LIO-NON-ROS.git
$cd source_directory
$mkdir build
$cd build
$cmake ..
$make
$./fastlio_mapping
```
> Before running the code, create "data" folder which contains your .bag file in .txt format. 
> You can use rosbag2txt.py in "scripts" folder for that purpose. 
> Don't forget to change path to the .txt file in "laserMapping.cpp" in line 31.

## To-do

Dillinger uses a number of open source projects to work properly:
1. Clean the code
2. Add the functionality of ros spin with defined frequency
