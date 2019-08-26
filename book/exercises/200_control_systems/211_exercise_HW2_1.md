# Exercise: Follow the Leader {#exercise-HW2_1 status=draft}

## Skills learned

* analyze a control task and design a suitable controller.
* implement a self-designed controller which integrates with the Duckiebot’s software.

## Following

In this exercise you will create your own follow-the-leader controller for the Duckiebot. A reference Duckiebot (leader) will drive ahead using lane following at reduced speed. Your task is to implement a controller for a second Duckiebot (follower) to follow the leader. Your controller function has the following inputs available (visualized in figure [](#fig:computer_vision)):

* $\rho$, the distance from the following Duckiebot to the reference Duckiebot;
* $\theta$, the angle between the following Duckiebot and the direct line connecting the following Duckiebot and the reference Duckiebot;
* $\psi$, the angle between the reference Duckiebot and the following Duckiebot;
* $t_{delay}$, the time it took between taking the image and the call of the controller function;
* $dt_{last}$, the time it took from the last call of the controller function until the current call.

To estimate $\rho$, $\theta$, and $\psi$, a computer vision algorithm that requires tags with black dots is used. While testing, make sure the tags are properly attached to the leader.

<center>
<figure>
    <img figure-id="fig:computer_vision" figure-caption="$\rho$, $\theta$ and $\psi$ as received by the computer vision algorithm." style="width: 70%; height: auto;" src="following.jpg"/>
</figure>
</center>

Design, implement and tune a follow-the-leader controller.

Note: The Dueckiebots can be prepared as described in [](#exercise-HW1_1). To start a leader, execute the following command on a Duckiebot with tags:

    duckiebot $ make csii-ex2-reference

To run the following Duckiebot use:

    duckiebot $ make csii-ex2-2
