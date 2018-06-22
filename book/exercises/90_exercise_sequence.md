
# Exercise: Git and conventions {#exercise-git-conventions status=draft}

TODO: move here the exercises we had last year about branching.




<!--  -->



<!--  -->



# Exercise: Use our API for arguments {#exercise-arguments-API status=draft}

## Skills learned

- Learn about the command-line API that we have in Duckietown,

## Instructions

We have a useful API that makes it easy to create programs with command line arguments.

TODO: to write

## Useful new API learned

* Our API for command line arguments.

<!--  -->

# Exercise: Bouncing ball {#exercise-bouncing-ball status=draft}

## Skills learned

* Show how to visualize data on a bag.
* Programmatic generation of images.
* Timestamps generation.

## Instructions

Create a program

    $ bag-bounce --fps ![fps] --speed ![speed]

that shows a bouncing ball on the screen, as if it were a billiard

## Useful new API learned {nonumber=1}

* Our API for command line arguments.

# Exercise: Visualizing data on image {#exercise-visualizing-data status=draft}

## Skills learned

* Show how to superimpose data on an image.


## Instruction

Write an implementation of `bag-mark-spots`.

## Specification for `bag-mark-spots`

Create a program that for each image, finds the pixels that are closest
to a certain color, and creates as the output a big red, yellow white spot on them.

    $ bag-mark-spots --input ![bag in] --mark "[[255,0,0],255,0,0]," --size 5 --output [bag out]


<!--  -->

# Exercise: Make that into a node {#exercise-make-node status=draft}

## Learned skills

* Abstracting code in interfaces that can be reused.
* Launch files.

## Instructions

Abstract the analysis above in a way that the same analysis code can be run equally
from a bag or on the laptop.

Make a ROS node and two launch files:

* One runs everything on the Duckiebot, and the output is visualized on the laptop.
* One runs the processing on the laptop.


<!--  -->

# Exercise: Instagram with EasyAlgo interface  {#exercise-instagram-easy-algo status=draft}

## Learned skills

* Use of our Duckietown API for abstracting algorithms.

## Instructions

TODO: Do the above using our API for filters.

We define the interface `InstagramFilter` and the EasyAlgo configuration files.

## See documentation

TODO: pointer to EasyAlgo

## Use in your code

Write a node using the EasyNode framework that decides which filters to run
given the configuration.

TODO: Also introduce the DUCKIETOWN_CONFIG_SEQUENCE.




# Milestone: Illumination invariance (anti-instagram) {#milestone-illumination-invariance status=draft}

TODO: Make them run our code, and also visualize what's going on


<!--  -->



# Exercise: Launch files basics {#exercise-launch-files status=draft}


## Learned skills

* Launch files



<!--  -->


# Exercise: Unit tests {#exercise-unit-tests status=draft}


## Learned skills

* Write unit tests that can be integrated in our framework.

## Unit tests with nosetests

- Unit tests

## Unite tests with ROS tests

- Integration with ROS tests

## Unite tests with comptests


# Exercise: Parameters (standard ROS api) {#exercise-parameters-ROS status=draft}

## Learned skills

- Reading parameters
- Dynamic modification of parameters

# Exercise: Parameters (our API)  {#exercise-parameters status=draft}

## Learned skills

- Use Duckietown API


# Exercise: Place recognition abstraction {#exercise-place-recognition status=draft}

## Learned skills

- ...


## Instructions

We use the following interface:

    class FeatureDescription():

        def feature(self, image):
            """ returns a "feature description" """

        def feature_compare(feature1, feature2):
            pass

We also provide a basic skeleton.

## Try a basic feature:

Simplest feature: average color.

    class AverageColor(FeatureDescription):

        def feature(self, image):
            return np.mean(image)

        def feature_mismatch(f1, f2)
            return np.abs(f1-f2)

Compute the similarity matrix for the

    $ similarity --feature average_color --input ![input.bag]  --output ![dirname]


## Bottom line

TODO: to write


# Exercise: Parallel processing  {#exercise-parallel-proc status=draft}

## Learned skills

* Do things faster in parallel.

## Instructions

We introduce the support for parallel processing that we have in the APIs.


# Exercise: Adding new test to integration tests  {#exercise-integration-test status=draft}

TODO: to write

## Learned skills

* Do things faster in parallel.

## Instructions

TODO: to write

## Bottom line

TODO: to write




## Milestone: Lane following

TODO: to write

# Exercise: localization {#exercise-localization status=draft}

TODO: to write


# Exercise: Ducks in a row {#exercise-duck-in-arow status=draft}

TODO: to write

# Exercise: Comparison of PID {#exercise-PIDs status=draft}

TODO: to write

# Exercise: RRT {#exercise-RRT status=draft}

TODO: to write
