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

<center>
![I'm not sure what's happening here](images/duckswap.jpg)
 
<br>
*I'm not sure what's happening here*
</center>

The world is inherently dynamic, but the algorithms we've looked at don't directly deal with dynamic obstacles. A naive solution is to, at each time-step, update your belief over the world-state (presumably reflecting the new position of dynamic objects in your environment) then regenerate your plan as though all obstacles are static. A bit more advanced solution may estimate the average velocity of dynamic obstacle across recent time-steps, then assume it will continue in the same way. The following work devises a way to remove egocentric movement from the motion prediction for dynamical obstacles.


<center>
![I'm not sure what's happening here](images/future_bounding_box.png)
<br>
Egocentric Vision-Based  Future Vehicle Localization for Intelligent Driving Assistance Systems<br>
*Yu Yao∗, Mingze Xu∗ , Chiho Choi, David J. Crandall, Ella M. Atkins, and Behzad Dariush*
</center>
<br>


Creating a good model of dynamic obstacles is much, much harder than it sounds. Humans mess this up all the time. Take, for example, the situation presented in the image above. You're trying to turn left, you have a green light, and no one is currently in your way. We see that the path-planning module, using its estimates of the oncoming vehicles position at previous timesteps, is predicting that the silver oncoming Mercedes is going to get in our way. But what if that driver is some kind of maniac and swerves in front of you? It happens. There's an enormous amount of unavailable information that must be modeled as uncertainty.

<br>
<center>
<br>
<br>
![](images/lambo.gif)
<br>
*We don't always know who's coming down the road, but sometimes it's this maniac in a lambo*
<br><br>
</center>

 To approach this problem we create an entirely deterministic situation in our simulator, add some stochastic behaviour to an oncoming duckiebot, create a probability distribution over their position, and use that distribution to inform our motion plan.



## Experimental Setup


To support this exercise, we designed a very simple 2-D (overhead view) simulator. It models a road with two lanes and two duckiebots. One is "ours" and the other is our adversary (lambo maniac) who we must avoid.

Similar to the setup in Duckietown, we have a small positive reward for staying in our lane, a small negative reward for crossing into the opposite lane, and larger negative rewards for leaving the road entirely or colliding with the other duckiebot.

Our solution builds a 4-dimensional tensor with two dimensions representing regions on our 2-dimensional map, a 3rd dimension represents time, and a fourth is the expected reward from being in that region at that time. We use the variant of Rapidly-exploring Random Trees (RRT) built on Dubin's curve, with Monte Carlo Tree Search (MCTS) to approximate our optimal trajectory up to a given time horizon.


## Installation
Prerequisites: ROS Kinetic, Lunar or Melodic, Python 2.7

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
The computation of the best trajectory is done in two parts:

1. Using the observations and a known movement model of the other Duckiebot, the Agent predicts the probability of collision at each time step for any x, y position of our Duckiebot.
2. Using Monte Carlo Tree Search with orientation change as the unique parameter, the agent finds the path with the highest reward (or lowest negative reward)


## Seeing what is happening

You can see what is happening by launching ```$ rqt_image_view``` in a new terminal and follow the topic ```/sim/road_image```. The red circle represents our Duckiebot, driven by the agent. The yellow circle represents the other duckie bot, driving according to the policy of the ```other_duckie_type``` specified in the sim parameters. The dark line in each circle represent their current heading.


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


%## Instructions

%Create a ROS node and package that takes as input the list of line segments detected by the line detector, and outputs an %estimate of the robot position within the lane to be used by the lane controller. You should be able to run:

%    duckiebot $ source ![DUCKIETOWN_ROOT]/environment.sh
%    duckiebot $ source ![DUCKIETOWN_ROOT]/set_vehicle.name.sh
%    duckiebot $ roslaunch dt_filtering_![ROBOT_NAME] lane_following.launch

%and then follow the instructions in [](+fall2017_info#checkoff_navigation) for trying the lane following demo.

%You should definitely look at the existing histogram filter for inspiration.

%You may find [this](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-%Filters.ipynb) a useful resource to get started.

%### Workflow Tip

%While you are working on your node and it is crashing, you need not kill and relaunch the entire stack (or even launch on your robot). You should build a workflow whereby you can quickly launch only the node you are developing from your laptop.


## Drive Safe

![](images/cool_duck.gif)
