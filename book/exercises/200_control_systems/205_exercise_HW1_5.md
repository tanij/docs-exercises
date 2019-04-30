# Exercise: Increasing the delay {#exercise-HW1_5 status=draft}

## Skills learned

* understand impacts of lower sampling rates.
* recognize issues associated to time delays.


## Basics

For now, we haven't talked explicitly about the delay which is present in our plant. However, from the moment an image on the camera is recorded until the control actions are applied, it takes roughly 70ms.

## Stability - Theory

As you've already seen, the time delay of 70ms does not destabilize our system. By using our calculations from [](#exercise-HW1_1)], we are indeed able to identify a maximal time delay such that our system is still stable in theory. This can be done by having a look at the transfer function of a time-delayed system:

$P_d(s) = e^{-sT_d} P(s)$

An increase of $T_d$ leads to a shift of the phase in negative direction. Therefore, $T_d$ must not be larger than the phase margin of $L(s)$ (which was roughly $70^{\circ}$) to not destabilize the system. Calculate the maximal $T_d$ such that the system is still stable.

## Stability - Practical

Before we can reach this theoretical limit, the Duckiebot will most likely leave the road and the estimator will fail since the lines are not in the field of sight of the camera anymore. By using **controller-5.py**, increase the time delay multiplicator $k_d$ of the system until the Duckiebot cannot stay in the lane anymore. Notice that the time delay is implemented in discrete steps of $k_d * T$ where T is the sampling time. As before, to modify the controller use:

    duckiebot $ make csii-edit-ex1-5

And execute it with

    duckiebot $ make csii-ex1-5
