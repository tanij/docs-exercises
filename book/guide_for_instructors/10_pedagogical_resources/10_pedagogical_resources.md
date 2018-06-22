# Pedagogical Resources {#pedagogical-resources status=draft}

A suite of pedagogical resources are provided that allow the an instructor to select how to present the material through different mechanisms in accordance with current educational principles
TODO: cite education paper
<!-- [](#bib:tani16duckietown). -->



## Background Materials

The background materials can be made available to students prior to the start of the class as and can be used to learn theory and build skills that will be useful in the class. 

### Theory Preliminaries

Theoritical background information is contained in the [](#book:preliminaries). 

### Software Reference

Software reference material is contained in the [](#book:sw-carpentry).


## Lecture Material

There are several different types of resources available [](#book:learning-materials) for use in classes and as learning aides for students. 

In general, the material is divided into "Units", where each unit is supported by some or all of the following: slides, written theory, python notebooks, exercises, and robot demonstrations.


### Slides

Slides are written directly in Markdown in this book. For example [](#autonomous-vehicles-slides). For information on how to create your own slides in this format see [](#making-slides).

Generally, which each slide presentation there is accompanying theory in the book in the same section. 


### Lecture Videos

Video lectures will be posted for each unit. See here for more details soon. 

TODO: Liam 


### Python Notebooks

Many units are also supported by notebook. One option is to present the lecture material and then run through the python notebook to reinforce the theory presented with practical examples.
The notebooks use real data logs, either from Duckietown or other sources to demonstrate the concepts presented in abstract form in the slides in a more concrete manner.

TODO: Liam

## Exercises

[](#book:exercises) are provided to help students master the material. They can optionally be used as graded homework assignments. Solutions are provided in a separate private [repository](https://github.com/duckietown/XX-exercises) that you should have access to if you are registered instructor of the course (follow the instructions on [here](www2.duckietown.org/guide-for-instructors). 



## Reproducible Robot Demonstrations

The backbone of the class and the entire project is the Duckiebot platform and the Duckietown environment. It is **required** that you build a Duckietown to support your class. Space can be an issue, but this is really the best part.

The instructions for building the Duckiebot are provided in [](#book:opmanual_duckiebot) and the instructions for building a conforming Duckietown are given in [](#book:duckietowns). The [](#book:opmanual_duckiebot) contains all the necessary information for assembly, calibration, troubleshooting, and running of the demos. The demos include basic [](#demo-lane-following), [](#sec:demo-indefinite-navigation) (the Duckiebot reads the road signs and takes a random feasible action),  and multi-robot operations such as [](#demo-coordination2017), and many others (the list continually grows).