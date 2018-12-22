# Exercise: Data collection for training {#exercise-data-collect status=ready}

<div class='requirements' markdown='1'>

Requires: You have built a [Duckietown](https://docs.duckietown.org/DT18/opmanual_duckietown/out/index.html) and set up a Vicon system as documented in [](#exercise-Vicon)

Requires: You have configured the [Duckiebot](http://docs.duckietown.org/DT18/opmanual_duckiebot/out/building_duckiebot_c0.html)

Result: Data for Visual Odometry training/testing

</div>

<div figure-id="fig:duckietown">
<img src="images/duckietown.jpg" style="width: 80%"/>
<figcaption>Duckietown in a Vicon area</figcaption>
</div>

## Installation
### Preliminaries
Make sure that you have a minimal environment, which includes:
  * Git
  * Catkin
  * ROS interface package compatible with VICON according to [](#ros-setup)
  * Matlab (optional)

### Data processing repo
Go to your workspace, clone and build [this repo](https://github.com/TienPoly/vo_duckiebot)



    laptop $ cd project_VO_ws/src
    laptop $ git clone https://github.com/TienPoly/vo_duckiebot.git
    laptop $ cd .. & catkin build


## Data collection
Run the base image on the Duckiebot

Create a ROS node and package that takes as input the list of line segments detected by the line detector, and outputs an estimate of the robot position within the lane to be used by the lane controller. You should be able to run:

    duckiebot $ source ![DUCKIETOWN_ROOT]/environment.sh
    duckiebot $ source ![DUCKIETOWN_ROOT]/set_vehicle.name.sh
    duckiebot $ roslaunch dt_filtering_![ROBOT_NAME] lane_following.launch

and then follow the instructions in [](+fall2017_info#checkoff_navigation) for trying the lane following demo.


    laptop-container $ docker -H ![hostname].local run -it --net host --privileged --name base -v /data:/data duckietown/rpi-duckiebot-base:master18 /bin/bash



When the container has started, publish images through ROS on the Duckiebot using the node `camera_node`


    container $ roslaunch duckietown camera.launch veh:="![hostname]" raw:="false"



Making your Duckiebot move


    laptop $ docker -H ![hostname].local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18



  *  Publishing Vicon data

    To write

  * Record data

    To write


  An example

## Decoder and synchronization
  * Decode image

  * Synchornization

  * Record new data




## Data export
  * Create images from bag file

  * Create txt file from bag
