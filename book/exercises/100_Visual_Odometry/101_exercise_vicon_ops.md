# Exercise: Using Vicon data {#exercise-Vicon status=beta}

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
<img src="images/Vicon_tracker_full.png" style="width: 60%"/>
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


### Frequency
To write

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
To write

### Check the Vicon Data
To write

## Vicon and ROS {#ros-setup status=ready}  
### Install ROS interface and dependencies
To write

### Network setup
To write

### Publishing Vicon data to the ROS Network
Launch the Vicon node
To write
