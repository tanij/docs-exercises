
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

In this package, we filter spurious lines and make odometry estimates smooth as well as remove the error of line detector which can cause of having lines which are too short or long, or they're not clear due to their distance from the robot.

This package takes in _ground-projected_ line segments and _filters out_ spurious lines. It subscribes to a topic that publishes a `SegmentList` message type, applies filters, and publishes the filtered line segments to another topic `filtered_segments_lsd` (again, as a `SegmentList` message type).

To ensure this node works, you need to set up the following topics in `lane-slam/src/line_sanity/launch/line_sanity.launch`. Assume your duckiebot is named *neo*.
1. By default, the `line_sanity_node` publishes to the topic `/neo/line_sanity_node/filtered_segments_lsd`. If you need it to publish it to another topic, open `line_sanity.launch` (in the `launch` directory of the `line_sanity` package). Around lines 5-7, where the `line_sanity_node` is being launched, add in a remap command.
```<remap from="~filtered_segments_lsd" to="name/of/topic/you/want/to/publish/to" />```
2. By default, the `line_sanity_node` subscribes to the topic `/neo/ground_projection/lineseglist_out_lsd`. If you need it to publish it to another topic, open `line_sanity.launch` (in the `launch` directory of the `line_sanity` package). Around lines 5-7, where the `line_sanity_node` is being launched, add in a remap command.
```<remap from="/neo/ground_projection/lineseglist_out_lsd" to="name/of/topic/you/want/to/subscribe/to" />```

*IMPORTANT:* This node will only work if the `SegmentList` that the node subscribes to has already been ground-projected. Else, there will be several `ValueError`s.

*Types of spurious lines filtered out:*
1. Lines behind the robot
2. Lines that are not white or yellow, and cannot be confidently classified as being left or right edges of a white or yellow line (we ignore RED lines for now).
3. Lines that are farther ahead from the robot, above a certain distance threshold.
4. All lines that do not satisfy a certain angular threshold.

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
