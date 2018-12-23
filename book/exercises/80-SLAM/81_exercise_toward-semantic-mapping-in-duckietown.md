
# Exercise: Towards Semantic Mapping in Duckietown {#semantin-mappping status=draft}

## Skills learned

* How to associate semantics to the roads in duckietown: represent the world in terms of lines and their colors
* Detect better lines compared to the current `lane_following` stack
* Use `odometry` information from virtually any source you would like to (wheel odometry, visual odometry, lane filter, etc.)
* Filter spurious lines and smooth odometry estimates
* Visualize the line-based semantic map built by your duckiebot
* Use what you know to make the map better

You can run all of this over your favourite log!

## Instructions

During the course we've seen different SLAM techniques here we're doing a lane-base SLAM using semantics as a prior knowledge.

### Semantic association to the road
To explain how we associate semantic to the lines

### Line Detection 
Explain using LSD 

### Wheel Odometry
How we build the map using wheel odometry 

### Line Filetring
Explain how we filter spurious lines and make smooth odometry estimates

## Image of expected results
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

The main Lane-SLAM repository is <a src="https://github.com/mandanasmi/lane-slam">here</a>. Clone the repo to visualize the lane-based semantic map that is used in the duckietown. We're detecting different types of lines in images (white, yellow and red) using LSD and then use discriptors to match lines and localize their position in an image. Then we do a wheel odometery to build the map.  

## Required packages

* Linux-based machines 

* OSx

* Windows



## Reproduce results 


## Set up the visualization


<!-- TODO: Validation and testing -->
