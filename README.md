# Niryo One Tester 

## Overview

This packages serves as example on how to communicate to the niryo one robot arm via ros, by establishing an action server connection. 

## Dependencies
This package depends on the actionlib and the niryo_one_msgs packages
#### actionlib 
Please refer to the ![documentation](http://wiki.ros.org/actionlib) of the actionlib package 
#### niryo_one_msgs
In your catkin worksapces source folder, do 

```
$ git clone https://github.com/NiryoRobotics/niryo_one_ros.git
$ catkin build
```
this will get you the whole niryo one ros stack

## Installation
In your catkin worksapces source folder, do 

```
$ git clone https://github.com/smaassen/niryo_one_tester.git
$ catkin build
```

## Included nodes
### simple_command_node
An example node that walks you through simple commands like got to pose (rpy, or qutarniion) and close/open gripper


## Credits
**Author(s)**: Steve Maassen   
**Maintainer**: Steve Maassen, smaassen@ethz.ch  
**Affiliation**: Autonomous Systems Lab, ETH Zurich / Disney Research Lab Zurich
