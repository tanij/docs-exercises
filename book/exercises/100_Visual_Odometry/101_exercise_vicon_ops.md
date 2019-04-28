# Exercise: Using Vicon data {#exercise-Vicon status=ready}

<div class='requirements' markdown='1'>

Requires: Vicon motion capture system including [Vicon Bonita](https://www.Vicon.com/products/archived-products/bonita "Vicon Bonita") infrared cameras, [Vicon Active Wand](https://www.Vicon.com/products/Vicon-devices/calibration "Vicon Active Wand") \(T-shaped\), Vicon software installed on a Vicon server and Wifi network.

Result: you can publish Vicon data (ground truth) of an object to the ROS network.

</div>

<figure class="flow-subfigures">  
    <figcaption>Vicon system</figcaption>
    <figure>
        <img style='width:6em'
        src='images/Vicon_cam.jpg'/>
        <figcaption>Vicon Bonita camera</figcaption>
    </figure>
    <figure>  
        <img style='width:18em' src='images/voliere.jpg'/>
        <figcaption>Vicon area with 12 Bonita cameras</figcaption>
    </figure>    
</figure>

## Vicon setup
This part will show briefly how to set up a Vicon system.

### Vicon Tracker
On the Vicon server (Windows), launch the _Vicon Tracker_ application and make sure that all the cameras are connected (green).

<div figure-id="fig:vicon_tracker">
<img src="images/vicon_tracker_full.png" style="width: 60%"/>
<figcaption>Vicon Tracker </figcaption>
</div>


### Calibration
Use the Active Wand to calibrate the Vicon system as in the current [Vicon Tracker User Guide](https://docs.Vicon.com/display/Tracker34/Tracker+documentation)

<figure class="flow-subfigures">  
    <figcaption>Vicon calibration</figcaption>
    <figure>
        <img style='width:12em'
        src='images/Vicon_tracker_calibration_calibrate.png'/>
        <figcaption>Calibration</figcaption>
    </figure>
    <figure>  
        <img style='width:14em' src='images/Vicon_active_wand.jpg'/>
        <figcaption>Vicon Active Wand</figcaption>
    </figure>    
</figure>

### Set the volume origin
Place the Active Wand at the desired origin and set it in Vicon Tracker

<div figure-id="fig:origin">
<img src="images/Vicon_origin.jpg" style="width: 15em"/>
<figcaption>Set the volume origin </figcaption>
</div>


### Vicon data sampling frequency
The default sampling frequency is 100Hz. You can change the requested frame rate up to 250Hz.

<div figure-id="fig:sam_freq">
<img src="images/vicon_freq.png" style="width: 15em"/>
<figcaption>Sampling frequency </figcaption>
</div>

The Vicon system is now ready to be used.

## Create a Vicon object
### Vicon marker placement
Place at least three [markers](https://www.vicon.com/products/vicon-devices/markers-and-suits) on the object that you want to track.

<!--
<div figure-id="fig:vicon_object" figure-class="flow-subfigures">
   <img style="width: 12em" src='images/vicon_good_object.jpg'   figure-id="subfig:vicon_good_object"  figure-caption="Good placement of markers"/>
   <img style="width: 12em" src='images/vicon_bad_object.jpg'   figure-id="subfig:vicon_bad_object"  figure-caption="Bad placement of markers"/>
   <figcaption>Example of Vicon marker placement for 2D object. Using the second marker placement</figcaption>
</div>  -->

<figure class="flow-subfigures">  
    <figcaption>Example of Vicon marker placement for 2D object. Why is the second marker placement (4 markers) not good? Answer: Due to symetrical structure, Vicon can not distinguish the head and the tail of the object. The position is always good but the heading angle can be deviated by 180 deg. </figcaption>
    <figure>
        <img style='width:13em' src='images/vicon_good_object.jpg'/>
        <figcaption>Good placement of markers</figcaption>
    </figure>
    <figure>  
        <img style='width:13em' src='images/vicon_bad_object.jpg'/>
        <figcaption>Bad placement of markers</figcaption>
    </figure>    
</figure>


### Create a Vicon object
Select the markers that define the object and click `Create` in the Objects tab of the `Vicon Tracker` application ([](#fig:create_obj)).

<div figure-id="fig:create_obj">
<img src="images/vicon_create_obj.png" style="width: 22em"/>
<figcaption>Creating an object </figcaption>
</div>

You can also change the origin and the orientation of the object as documented [here](https://docs.vicon.com/display/Tracker33/About+the+Objects+tab). Vicon  uses the standard engineering coordinate system of $x$ axe - forward (Red), $y$ axe - right (Green), $z$ axe - up (Blue).


### Check the Vicon Data
Finally, you can track your object with Vicon and trace its pose as shown in [](#fig:graph)

<div figure-id="fig:graph">
<img src="images/vicon_graph.png" style="width: 22em"/>
<figcaption>Tracking an object </figcaption>
</div>


## Vicon and ROS {#ros-setup status=ready}  
### Install ROS interface and dependencies
You can use the ROS interface for [VRPN Client](http://www.cs.unc.edu/Research/vrpn/).

Go to your workspace, clone and build this repo and its dependencies:



    laptop $ cd ![your_ws]/src
    laptop $ git clone https://github.com/MRASL/ros_vrpn_client
    laptop $ git clone https://github.com/ethz-asl/vrpn_catkin
    laptop $ git clone https://github.com/catkin/catkin_simple.git
    laptop $ git clone https://github.com/ethz-asl/glog_catkin.git
    laptop $ cd .. & catkin build



### Publishing Vicon data to the ROS Network
Run the node `vrpn_client` using the launch file `mrasl_vicon_duckiebot`


    laptop $ roslaunch ros_vrpn_client mrasl_vicon_duckiebot.launch object_name:=![vicon_object_name]


This launch file is a copy of the original `asl_vicon.launch`, using for the object `vicon_object_name` and the Vicon server IP 192.168.1.200.

Using `rostopic list`, you can see the following topics from Vicon:


    /duckiebot_razor/vrpn_client/estimated_odometry                                                      /duckiebot_razor/vrpn_client/estimated_transform                                                     
    /duckiebot_razor/vrpn_client/raw_transform                                                           
    /duckiebot_razor/vrpn_client/vicon_intermediate_results                                              
    /rosout                                                                                               
    /rosout_agg         
