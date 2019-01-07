# Exercise: Data collection for training {#exercise-data-collect status=ready}

<div class='requirements' markdown='1'>

Requires: You have set up a Vicon system as documented in [](#exercise-Vicon)

Requires: You have built a [Duckietown](https://docs.duckietown.org/DT18/opmanual_duckietown/out/index.html) in a Vicon area

Requires: You have configured the [Duckiebot](http://docs.duckietown.org/DT18/opmanual_duckiebot/out/building_duckiebot_c0.html)

Result: Data for Visual Odometry training/testing including

* Raw data bags: compressed images at 30Hz and Vicon data at 90Hz, as [here](https://drive.google.com/drive/folders/1qHwnsSbwiy0KzJFvdVZtMMPqe27wOrzn)
* Synchronized data bags: raw images and ground truth at about 10Hz, as [here](https://drive.google.com/drive/folders/1UK7oukmLbaMgxJrnhZbbOy8kXzSsohAz)
* Compatilble data for Deep learning: [images](https://drive.google.com/drive/folders/1gHycYIOBIMTqOWKfAXXuZZgXMw5jXxb2) and [ground truth](https://drive.google.com/drive/folders/19HOO3IqiI70Ay0XJ7LQXO7wie6DOsFcS)

</div>

<div figure-id="fig:duckietown">
<img src="images/duckietown.jpg" style="width: 80%"/>
<figcaption>Duckietown in a Vicon area</figcaption>
</div>


## Preliminaries
First, make sure that you have a minimal environment, which includes:

1. Git
2. Catkin
3. ROS interface package compatible with VICON ([](#ros-setup))
4. Matlab (optional).

Go to your workspace, clone and build [this repo](https://github.com/TienPoly/vo_duckiebot)



    laptop $ cd ![your_ws]/src
    laptop $ git clone https://github.com/TienPoly/vo_duckiebot.git
    laptop $ cd .. & catkin build


## Raw Data collection
### Camera
Run the base image on the Duckiebot


    laptop $ docker -H ![hostname].local run -it --net host --privileged --name base -v /data:/data duckietown/rpi-duckiebot-base:master18 /bin/bash



When the container has started, publish images through ROS on the Duckiebot using the node `camera_node` (at about 30Hz)


    laptop-container $ roslaunch duckietown camera.launch veh:="![hostname]" raw:="false"


You can verify the output by using the ROS utilities:


    laptop $ export ROS_MASTER_URI=http://![hostname].local:11311/
    laptop $ rqt_image_view


### Joystick
Making your Duckiebot move as documented [here](http://docs.duckietown.org/DT18/opmanual_duckiebot/out/rc_control.html)


    laptop $ docker -H ![hostname].local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18


### Vicon
Publish Vicon data according to [](#ros-setup) (at about 90Hz to ensure that the ground truth information is always available for each image)

    laptop $ export ROS_MASTER_URI=http://hostname.local:11311/
    laptop $ roslaunch ros_vrpn_client mrasl_vicon_duckiebot.launch object_name:=![vicon_object_name]


### Record data

    laptop $ export ROS_MASTER_URI=http://hostname.local:11311/      
    laptop $ rosbag record /![hostname]/camera_node/camera_info  /![hostname]/camera_node/image/compressed /![vicon_object_name]/vrpn_client/estimated_odometry

Example of raw data bags can be found [here](https://drive.google.com/drive/folders/1qHwnsSbwiy0KzJFvdVZtMMPqe27wOrzn).

## Decoding and synchronization
### Raw images
Due to limited computation, the node `decoder_node` is run on the Duckiebot at very low frequency (2Hz). To get more images for deep learning, you have to run this node on a local desktop.

Run the `roscore` node and play back the content of your raw data bag (another terminal)



    laptop $ roscore
    laptop $ rosbag play ![your_bag].bag


    <!--  --topic /hostname/camera_node/image/compressed  /duckiebot_hostname/vrpn_client/estimated_odometry  -->


Run the node `decoder_node` at 10Hz

    laptop $ cd `your_ws` && source devel/setup.bash
    laptop $ roslaunch vo_duckiebot decoder_node.launch veh:="![hostname]" param_file_name:="decoder_10Hz"


You can specify the parameter `param_file_name` to publish raw images at other frequencies.


### Synchornization
Run the node `data_syn_node` to subscribe and republish the raw images and the ground truth information at the same frequency (as soon as new raw image becomes available)

    laptop $ cd project_VO_ws && source devel/setup.bash
    laptop $ roslaunch vo_duckiebot data_syn.launch veh:="hostname" veh_vicon:="duckiebot_hostname"


### Record new data

    laptop $ cd `your_ws` && source devel/setup.bash
    laptop $ rosbag record /![hostname]/camera_node/image/raw /![hostname]/vicon_republish/pose

Example of synchronized data bags can be found [here](https://drive.google.com/drive/folders/1UK7oukmLbaMgxJrnhZbbOy8kXzSsohAz)


## Data export
### Extract images from your bag file
Run the Python script `bag2img.py` in the folder `![your_ws]/src/vo_duckiebot/tools/` using your [bag file](https://drive.google.com/drive/folders/1UK7oukmLbaMgxJrnhZbbOy8kXzSsohAz).

    laptop $ cd ![your_ws]/src/vo_duckiebot/tools/
    laptop $ ./bag2img.py ![your_bag].bag ![your_image_folder]/ /![hostname]/camera_node/image/raw


Example of extracted images can be found [here](https://drive.google.com/drive/folders/1gHycYIOBIMTqOWKfAXXuZZgXMw5jXxb2)

### Create text files (ground truth information) from your bag file
Run the Matlab script `script_to_run.m` in the folder `![your_ws]/src/vo_duckiebot/tools/` with your [bag file](https://drive.google.com/drive/folders/1UK7oukmLbaMgxJrnhZbbOy8kXzSsohAz).

Example of text files can be found  [here](https://drive.google.com/drive/folders/1AwUa0nw_ARssUREuY_LAfms_CEQIHvGP)

## Modified heading angle
### Quaternion to Euler angles problem
The `atan2` function is used to calculate the Duckiebot heading angle from the quaternion given by Vicon. Therefore, the range of this angle is $[-\pi,\pi]$. This constraint can cause a data jump from $\pi$ to $-\pi$ as shown in [](#fig:angle) of the test case [alex_3small_loops](https://drive.google.com/drive/folders/19HOO3IqiI70Ay0XJ7LQXO7wie6DOsFcS).

<div figure-id="fig:angle">
<img src="images/alex_3small_loops.jpg" style="width: 90%"/>
<figcaption>Modified heading angle</figcaption>
</div>


### Modified yaw angle
When there is a data jump, the heading angle is modified by adding $\pm\pi$. The modified angle is now continuous ([](#fig:angle)) and compatible for the training process.
