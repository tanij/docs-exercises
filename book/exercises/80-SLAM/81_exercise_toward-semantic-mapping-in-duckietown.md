
# Exercise: Towards Semantic Mapping in Duckietown {#semantin-mappping status=draft}

## Skills learned

* How to associate semantics to the roads in duckietown: represent the world in terms of lines and their colors
* Detect better lines compared to the current `lane_following` stack
* Use `odometry` information from virtually any source you would like to (wheel odometry, visual odometry, lane filter, etc.)
* Filter spurious lines and smooth odometry estimates
* Visualize the line-based semantic map built by your duckiebot
* Use what you know to make the map better

You can run all of this over your favourite log!

## Expected results
<figure>
    <figcaption>Examples of expected visualization: </figcaption>
	<figure>
	    <figcaption>Map 1.</figcaption>
	    <img style='width:20em; height:15em' src="figures/map1.png"/>
	</figure>
	<figure>
	    <figcaption>Map 2.</figcaption>
	    <img style='width:20em; height:15em' src="figures/map2.png"/>
	</figure>
</figure>
## Demo Code


## Instructions
The main Lane-SLAM repository is <a src="https://github.com/mandanasmi/lane-slam">here</a>. Clone the repo to visualize the lane-based semantic map that is used in the duckietown. We're detecting different types of lines in images (white, yellow and red) using LSD and then use discriptors to match lines and localize their position in an image. Then we do a wheel odometery to build the map.  

During the course we've seen different SLAM techniques here we're doing a lane-base SLAM using semantics as a prior knowledge. For that, we defined several nodes and packages that we described below: 


### Line Detector

This package is an enhanced version of the `line_detector` from the Duckietown `lane_control` stack. This node subscribes to a topic that publishes a `CompressedImage` message type, detects line segments in that image, and publishes the detected line segments, their normals, color, and position information in the form of a `SegmentList` message.

*Our modifications:* We adapt the original `line_detector` package to include a new line segment detector [LSD](https://docs.opencv.org/3.4/db/d73/classcv_1_1LineSegmentDetector.html). This detector gets far more stable  and longer line segments compared to `HoughLines`, which seems to be the default.

*Example usage:*

To set up all you need to run the node, assume your duckiebot name is *_neo_*. _Neo_ publishes camera info to a topic named `/neo/camera_node` (i.e., the topic that `line_detector_node` will subscribe to is `/neo/camera_node/image/compressed`). It also has a bunch of line detector parameters are available in the file `lane-slam/src/duckietown/config/baseline/line_detector/line_detector_node/default.yaml`

To run this node using `rosrun`, execute the following command.
`roslaunch line_detector line_detector_node.launch --veh:neo --local:=true` (on your laptop)

### Line Descriptor

This package uses _OpenCV_ functions to compute binary descriptors for a bunch of line segments, to help in matching/associating lines. Currently, this functionality is in beta, but people are welcome to play around with the code in here.

### Line Associator



### Line Sanity

In this package, we filter spurious lines and make odometry estimates smooth as well as remove the error of line detector which can cause of having lines which are too short or too long.


### Odometry


### Show Map


### All in one



## Required packages

* Linux-based machines 

* OSx

* Windows


## Reproduce results 


## Set up the visualization


<!-- TODO: Validation and testing -->
