# Exercise: Motion Planning with Uncertainty {#motion-planning-with-uncertainty status=ready}

Assigned: Martin Weiss, Gunshi Gupta, Vincent Mai


## Pre-Requisites

* An optimistic outlook on life
* ROS Kinetic, Lunar, or Melodic, Python 2.7, catkin, scipy
* A basic understanding of path planning


## Skills learned

* How to use a very simple simulator that we designed with ROS and python
* Advanced Motion-Planning techniques


## Introduction to Motion Planning with Uncertainty

During the lectures, we saw many classic methods of motion planning such as graph-search methods, variational methods, and incremental search methods. The goal of motion/path-planning is generally to get from A to B without crashing, using too many resources (time, gas), or otherwise violating some specified constraints. Most systems that we're interested are non-holonomic, where the velocities (magnitude and or direction) and other derivatives of the position are constrained by some dynamics of the vehicle (e.g. your truck has a minimum turning radius).

We are going to focus on a significantly more difficult problem in this tutorial, planning under uncertain conditions. There are many kinds of uncertainty including uncertainty over your initial state, your action model, and your noisy sensors.


![I'm not sure what's happening here](images/duckswap.jpg)
<br>
*I'm not sure what's happening here*

The world is inherently dynamic, but the algorithms we've looked at don't directly deal with dynamic obstacles. A naive solution is to, at each time-step, update your belief over the world-state (presumably reflecting the new position of dynamic objects in your environment) then regenerate your plan as though all obstacles are static. A bit more advanchttps://github.com/mweiss17/docs-exercisesed solution may estimate the average velocity of dynamic obstacle across recent time-steps, then assume it will continue in the same way. The following work devises a way to remove egocentric movement from the motion prediction for dynamical obstacles.


![I'm not sure what's happening here](images/future_bounding_box.png)
<br>
Egocentric Vision-Based  Future Vehicle Localization for Intelligent Driving Assistance Systems<br>
*Yu Yao∗, Mingze Xu∗ , Chiho Choi, David J. Crandall, Ella M. Atkins, and Behzad Dariush*
</center>
<br>


Creating a good model of dynamic obstacles is much, much harder than it sounds. Humans mess this up all the time. Take, for example, the situation presented in the image above. You're trying to turn left, you have a green light, and no one is currently in your way. We see that the path-planning module, using its estimates of the oncoming vehicles position at previous timesteps, is predicting that the silver oncoming Mercedes is going to get in our way. But what if that driver is some kind of maniac and swerves in front of you? It happens. There's an enormous amount of unavailable information that must be modeled as uncertainty.

![](images/lambo.gif)
<br>
*We don't always know who's coming down the road, but sometimes it's this maniac in a lambo*
<br>

 To approach this problem we created an entirely deterministic situation in our simulator, added some stochastic behaviour to an oncoming duckiebot, created a probability distribution over their velocity, and use that distribution to inform our motion plan.



## Experimental Setup


To support this exercise, we designed a very simple 2-D (overhead view) simulator. It models a road with two lanes and two duckiebots. One is "ours" and the other is our adversary (lambo maniac) who we must avoid.

Similar to the setup in Duckietown, we have a small positive reward for staying in our lane, a small negative reward for crossing into the opposite lane, and larger negative rewards for leaving the road entirely or colliding with the other duckiebot.

### Problem we want to solve:

Imagine you are driving on a straight road, in your lane, when you suddenly see in front of you a big truck driving on the middle of the road. It is so big, that you have no choice but to avoid it by partially going out of the road. The thing is, you know that the guy driving is a bit sleepy: he does not touch the wheel, but he plays with the accelerator and the break pedal randomly. How do you plan your trajectory to avoid collision while minimizing your driving partially out of the road?
This problem gives us two possible cases: when the truck is coming towards us, or when it is going in the same direction than us. The solution should be able to perform well in these two cases.

### Our solution:

Our solution predicts the probability of the other duckiebot's position in time, using the velocity probabilistic model to propagate its possible positions over a one-dimensional discretized map. It is then able to compute the probability of collision for our Duckiebot at any position and time. This is used by a Monte Carlo Tree Search algorithm in order to compute a probable reward at each node, and to approximate our optimal trajectory up to a given time horizon.

### The exercise:

Can you do better than us? In `include/dt_agent`, the file `agent.py` is instantiating a predictor and a planner. Add your own predictor and planner in the same folder, and don't forget to include them in the `include/dt_agent/__init.py__`. Compare with our version by measuring your score at t = 25 sec.
But first, you'll have to install the whole thing. The following sections explains you how to, and then, how does the whole package work.


## Installation
Prerequisites: ROS Kinetic, Lunar or Melodic, Python 2.7, catkin, scipy

In `catkin_ws/src`, clone the package, then:

```$ catkin_make```

Source the setup file: ``` $ source ./devel/setup.bashrc```

And finally, if you are on Linux, you may need set the python files as executables:
```$ cd src/pathplan_uncertainty/src```
```$ sudo chmod +x dt_*```

## Launching full demo
The whole demo can be launched via the terminal:

```roslaunch pathplan_uncertainty dt_pathplan.launch```

It launches the simulator, the manager, and the agent nodes together.


## Simulator node

### Purpose
The simulator node allows to simulate the movement of two Duckiebots: our Duckiebot (controlled by the agent node) and the other Duckiebot (controlled by a pre-determined stochastic policy). It has perfect information about our current position, orientation, safety status (i.e. was there a collision) and lane position (right lane, left lane, partially out of the road, completely out of the road). N.B. that we use lane position and ground_type synonymously.

### Launch
The simulator ros node can be launched on its own with this command:

```roslaunch pathplan_uncertainty dt_simulator_node.launch```

### Test
Can be tested using rqt to simulate an agent. Example here:

While node is running, in a new terminal, run `$ rqt`

From the top menu, open Plugins/Topics/Message Publisher.

From the developing menu, choose:
`/agent/agent_command`, add it with a freq of 1 Hz.
Develop the topic and give a value of 5 to `compute_time_steps` (for example) and  `[0, 0, 0, 0, 0]` to `orientation_seq`.
Check the checkbox to start publishing.
The simulation should run with our DuckieBot having a 5 time-step straight trajectory.

### Communication

#### Listening to this topic:
  * ```/agent/command```, including:
    * ```computation_time_steps```: _k_ time steps the agent will take to compute the next trajectory.

    * ```orientation_seq```: trajectory to be executed in the meantime (only orientation, in radians).

Once the Simulator receives a message on this topic, it publishes an observation and starts executing the trajectory over the _k_ time steps.


#### Publishing on these topics:
This topic should be listened to by the Agent:

  * ```/sim/obs/observations``` includes both Duckies' poses, their velocities, and their radii. It is sent **after the _k_ time steps have been executed**.

This topic should be listened to by the Manager and is published **at every time step**:

  * ```/sim/gt/world_state``` contains the time, both Duckies' poses, our Duckie's safety status and the type of ground on which our Duckie is.

For debugging purposes, the node also publishes these topics **at every time step**:

  * ```/sim/gt/pose_our_duckie``` is the pose of the our Duckie.

  * ```/sim/gt/pose_other_duckie``` is the pose of the other Duckie.

  * ```/sim/gt/our_duckie_ground_type``` is the ground type on which our Duckie is.

  * ```/sim/gt/our_duckie_safety_status``` is the safety status of our Duckie.

The ground type and safety status are encoded according to the protocole defined in `/config/communcations.yaml`

For visualization purposes, the node also publishes this topic:

```/sim/road_image``` is an image showing the current state (more details in the *Seeing what is happening* section)

#### Service
The ```get_ground_type``` service can be called to return the ground type on which a robot with a given position and radius would be.

### Technical details

#### Coordinate system
We use a fixed coordinate system (it does not follow our Duckiebot).
  * *x* is the lateral coordinate, raising from left to right. It is 0 at the center of the right lane. It is in meters.
  * *y* is the vertical coordinate., raising from down to up. It is 0 at the initial position of our Duckiebot. It is in meters.
  * *theta* is the angular coordinate. It is 0 when looking up, *-pi*/2 when looking left, *pi* when looking down and yes, * pi/2* when looking right. Yes, we know, it is the other way around than the usual definition. It was made "on purpose" to reward those who read the instructions completely. It is in radians.

#### How is our Duckiebot movement encoded?

When the `orientation_seq` message arrives, it contains a sequence of orientations in radians. We consider that our Duckiebot has a constant speed and cannot accelerate nor break. Therefore, at each time step, it will follow the orientation given by `orientation_seq`. 
Note that the Duckiebot cannot turn extremely fast. Therefore, we limit its orientation change at every time step: if the difference between the orientation command and the current orientation is bigger than the limit, the orientation change is set to the limit.

#### How is the other Duckiebot movement working?

The other Duckiebot is going straight forward. Remember: it is a truck and the sleepy driver does not touch the wheel. However, the driver plays with the accelerator and the breaks. We model this by drawing, at every time step, an acceleration from an uniform distribution. The truck is not unlimited though: it has a minimum and a maximal velocity, from which it cannot respectively break or accelerate anymore.

## Manager Node
### Purpose
The Manager tracks the evolution of the simulation. It records the state at every time step and computes the score.

### Launch
Can be launched using:

```$ roslaunch pathplan_uncertainty dt_manager_node.launch```

### Communication

#### Listening to this topic:
  * `/sim/gt/world_state` contains the time, both Duckies' poses, our Duckie's safety status and the type of ground on which our Duckie is.

Once the Manager receives this message, it records the data and computes the reward and score of the Duckiebot. Then, it publishes the score.

#### Publishing on this topic:
  * `/manager/current_score` is the current score of the Duckiebot, published at every time step.

### Service
The `/manager/get_manager_records` service can be called to return the whole record of states.


## Agent Node
### Purpose
The Agent receives observations from the simulator and controls the actions of our Duckiebot. Because this whole program works in discrete time steps, the Agent reports its computation time to the Simulator and incurs a lag before updated path plans take effect on our duckiebot.

### Launch
Can be launched using:
``` $ roslaunch pathplan_uncertainty dt_agent_node.launch```

### Communications

#### Listening to this topic:
  * `/sim/obs/observations` : includes both Duckies' poses, their velocities, and their radii.

Once the Agent receives this message, it computes the best trajectory, simulates its computation time, and publishes them.

#### Publishing on this topic:
  * `/agent/command`, including:
    * `computation_time_steps`: _k_ time steps the agent will take to compute the next trajectory.

    * `orientation_seq`: trajectory to be executed in the meantime (only orientation, in radians).

### Computation of the trajectory
In our case, the computation of the best trajectory is done in two parts:

1. Using the observations and a known movement model of the other Duckiebot, the Agent predicts the probability of collision at each time step for any x, y position of our Duckiebot. This is done in two steps:
  a. Given the position, heading and velocity of the other Duckiebot, the Predictor uses its probabilistic velocity model to approximate the probability of the other Duckiebot's position (in the *y* axis only) at every time step until a given horizon. This is saved on a discretized map.
  b. Then, given a x, y position for a time step, the Predictor checks every *y* point on the discretized map that would lead into a collision, and add the probabilities of the other Duckiebot to be there to give the probability of collision.
2. Using Monte Carlo Tree Search with orientation change as the unique parameter, the agent finds the path with the highest reward (or lowest negative reward).

## Timing

You will have noticed that the nodes do not run in real time. Instead, it is a simple loop.

### How does it work?
1. To kickstart the process, the Agent sends an empty command with *k = 1* computation time step.
2. The Simulator sends the observations to the Agent and then propagates the world for *k = 1* time step with an empty command (our Duckiebot does not move).
3. The Agent receives the observations. It computes the next trajectory and sends it with a random computation time step number *k*.
4. The Simulator receives the trajectory and *k*. It sends the observations to the Agent and then propagates the world for *k* time steps with the given trajectory.
5. The Agent receives the observations. Repeat stage 3, then 4, until program is stopped using Ctrl+C

At every time step, the Simulator also publishes the ground truth state so that the Manager can record the trajectory and count the score. It also publishes the image so that we humans can visualize what is happening. In order for the image sequence to make sense, the Simulator waits a bit after having published each image.

### Why did we do it like this?
The computation cost of the optimal trajectory can be heavy. It may not be realistic to expect it to be done at every time step. In the real world, as soon as the Agent is done computing, it sends it to the actuators, observes the world, and starts computing again with the new informations. This is the way it is modeled in this case.

## Seeing what is happening

You can see what is happening by launching ```$ rqt_image_view``` in a new terminal and follow the topic ```/sim/road_image```. The red circle represents our Duckiebot, driven by the agent. The yellow circle represents the other duckie bot, driving according to the policy of the ```other_duckie_type``` specified in the sim parameters. The dark line in each circle represent their current heading. The white curve represents the trajectory that our Duckiebot has decided to follow for the next few time steps.


## Parameters
Different parameters files are called by the different nodes. They can be found in the `/config/` directory.

In `sim.yaml`, you will find:
  * `dt` : the value in seconds of each time step in the simulation
  * `dt_in_sim` : the value in seconds during which the simulation waits at each time step (allows the video to be real time)
  * `image`: parameters of the image, including its height, width, meter to pixel ratio, and the baseline in pixels from which y=0 is displayed, from the bottom of the image. You can disable the image output by setting `output_image` to False.
  * `world`: world parameters. Mainly, the width of the whold road, in meters (each lane is therefore half of the road width).
  * `other_duckie_type` : the type of the other Duckiebot. Can be set to `constant_speed_duckie` or to `unstable_speed_duckie`

In `duckiebots.yaml`, you will find, for each Duckiebot type:
  * `start_pose` : the pose at start (x, y, theta) with x, y in meters, x = 0 the center of the right lane, and theta in radians, theta = 0 looking forward.
  * `velocity` : the velocity at start, in m/s
  * `radius` : the radius of the Duckiebot, in meters - we consider the Duckiebot to be a circle
  * `type` : the type of the Duckiebot
  * `angle_change_limit` : the maximal change in orientation between two time steps, in radians
  * `max_acceleration` : the maximal acceleration in a second that the Duckiebot can undertakȩ
  * `max_velocity` and `min_velocity` : the maximal and minimal velocities of the Duckiebot, in m/s

In `rewards.yaml`, you will find the rewards given at each time step:
  * `status_fine` : while there is no collision
  * `status_collision` : when there is a collision
  * `type_right_lane`: when the Duckiebot is in the right lane
  * `type_wrong_lane`: when the Duckiebot is in the wrong lane
  * `type_partially_out`: when the Duckiebot is partially out of the road
  * `type_lost`: when the Duckiebot is completely out of the road

In `communications.yaml`, you will find the communication protocole used by the nodes to exchange information on the safety status and the ground type on which the Duckiebot is.

In `agent.yaml`, you will find the parameters used by the agent to predict, plan, and simulate the computation time:
  * `predictor` :
    * `time_horizon` : time in seconds until which the predictor predicts the position probability of the other Duckiebot
    * `y_resolution` : resolution in meters of the discretization of the y dimension for the predictor
    * `y_horizon` : distance in meters from the origin until which the predictor runs
    * `vel_resolution` : resolution in m/s of the velocity change discretization. For example, with `dt` = 0.2s, and an uniformly distributed acceleration between -`max_acceleration` and + `max_acceleration` with `max_acceleration` = 2 m/s^2, the predictor will at each time step consider the possible velocity changes being of [-0.4, -0.2, 0, 0.2, 0.4] m/s, each of them with a 0.2 probability.
  * `computation_time` :
    * `mean` : average number of time steps taken to simulate the computation time
    * `std_dev` : standard deviation in time steps of the simulated computation time
  * `mcts` :
    * `scalar` : raise this number to increase the exploration behavior, lower the number to increase the exploitation behavior of the MCTS
    * `budget` : Iterations of the MCTS through the tree. In each iteration, a new node is added so the tree is expanded.
    * `time_steps` : amount of time steps the MCTS is looking ahead

## Our results
Our solution works well. Kind of. Here are some results we would like to share, and a bit of analysis over the issues we are having.

### Gifs
These gifs show the output of the visualizer with different MCTS parameters. We took the case of the truck coming towards us, with a random acceleration picked from a continuous uniform distribution from -2 m/s to + 2 m/s at each time step. The trajectory was recomputed every 3 time steps.

#### With default parameters
![](images/default-params-initial.gif)

MCTS parameters:
  * scalar: 1.414
  * budget: 5300
  * time_steps: 50
  
#### With 20 time steps
![](images/20timesteps.gif)

 MCTS parameters:
  * scalar: 1.414
  * budget: 5300
  * time_steps: 20

#### With a budget of 2000 iterations
![](images/2000-iterations.gif)

MCTS parameters:
  * scalar: 1.414
  * budget: 2000
  * time_steps: 50

#### Increasing the explore parameter to 2
![](images/explore-parameter-2.gif)

MCTS parameters:
  * scalar: 2
  * budget: 5300
  * time_steps: 50

### Analysis
#### Fluidity of the gifs
the gifs are not very fluid. Why? In order to accelerate the gifs, we removed the waiting time at each time step. Therefore, the Simulator goes very quickly through 3 time steps, and then waits for the Agent to compute the next trajectory.

#### Variable trajectory length
The trajectory length is not constant. Sometimes, it is very short, sometimes, very long. This is because the depth of the tree changes depending on the region it is being optimised in. In some regions, the tree can have a lot of children with competitive reward possibilities, so the tree becomes wider, but less deep. Therefore, the optimal path might have a different length.

#### Avoiding the Duckiebot
The agent always avoids the other Duckiebot! That's great - it shows that our algorithm is working!

#### Avoiding to get out of the road
However, our agent somehow decides to get completely out of the road. This is bad, because we set the negative reward of being completely out of the road as bad as having a collision! Additionnally, it is not necessary to avoid the other Duckiebot. What is happening here?

Our analysis of the problem is that our MCTS does not explore enough at the very beginning. When the agent plans to go out of the road on a long term trajectory, it correctly does not get completely out of the road. However, when the agent follows this trajectory, it has at some point to be partially out of the road and looking to the right. Its trajectory plans that the next step will be turning on the left, and all will be fine.But, just when it gets there, the trajectory is completely recomputed! Unluckily, the agent chooses to simply go forward as a first step instead of exploring the other directions. This leads to the agent getting lost in the wild... for a small amount of time, until it gets back on the road. 

This problem could be reduced by reducing the time step duration, increasing the exploration or increasing the amount of time steps for which the trajectory is followed. However, the two first options induce a higher computation time for the agent, whereas the last option means driving with more uncertainty, which adds to the probability of a collision.

#### Influence of the parameters
Frankly, we find that the 4 experiments look pretty similar. The parameters change a bit the behavior: a lower number of time steps lowers the length of the trajectory, a higher scalar makes the trajectory move a bit more and a smaller tree depth. But overall, they do not affect the performance so much. We would love to have a more complete analysis to show you, but we hope you will agree with us!

## Drive Safe

![](images/cool_duck.gif)
