# Exercise: LQRI controller {#exercise-HW3_3 status=draft}

## Skills learned

* implement an LQRI-controller.
* understand impacts of weights on control performance and tracking errors.

## LQRI controller
Since the LQR controller has no integrator action, a steady state error will persist. To eliminate the steady state error, a LQRI controller will be implemented. An integral state needs to be calculated and is additionally used as a state feedback (figure [](#fig:lqri)).

<center>
<figure>
    <img figure-id="fig:lqri" figure-caption="Block diagram of a LQRI controller." style="width: 80%; height: auto;" src="lqri.jpg"/>
</figure>
</center>

The state feedback matrices $K$ and $K_I$ can be calculated by solving the Discrete-time Algebraic Riccati Equation (DARE) using the extended state space matrices.


Extend the continuous state space model of the Duckiebot by a state that describes the integral of the distance. Discretize the new state space matrices as in [](#exercise-HW3_1). Pay attention to the appropriate dimensions of your matrix. Why is no integral state of the angle used?


Implement a LQRI-controller in the file **controller-lqri.py**.  Start from the LQR-controller you designed in [](#exercise-HW3_2). Extend the state space matrices and the weighting matrices. Since your controller function does not get the new integral state as an input,you have to calculate it within the function.


Note:If you don’t know how to calculate the integral state, have a look at the PI-Controller of [](#exercise-HW1_2).
