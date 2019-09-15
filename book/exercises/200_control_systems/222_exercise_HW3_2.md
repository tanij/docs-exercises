# Exercise: LQR controller {#exercise-HW3_2 status=draft}

## Skills learned

* implement a steady-state Linear Quadratic Regulator (LQR).
* understand impacts of weights in the cost function on control performance.


## LQR controller
To achieve a better lane following behavior a LQR controller can be implemented. The structure of a state feedback controller is:

<center>
<figure>
    <img figure-id="fig:lqr-feedback" figure-caption="Block diagram of a state feedback control." style="width: 70%; height: auto;" src="lqr_feedback.png"/>
</figure>
</center>

Because of limited computation resources, we implement a stead-state (or _infinite horizon_) version of the LQR. Because we are considering the discrete time model of the Duckiebot, the Discrete-time Algebraic Riccati Equation (DARE) has to be solved:

$\Phi = A^T\Phi A - (A^T \Phi B)(R+B^T \Phi B)^{-1}(B^T \Phi A)+Q$

To solve this equation use the Python control library (see [python-control.readthedocs.io](https://python-control.readthedocs.io/en/0.8.2/)for documentation).

<center>
<figure>
    <img figure-id="fig:duckiebottopview" figure-caption="Top view of the Duckiebot on the track." style="width: 40%; height: auto;" src="duckiebot_topview.pdf"/>
</figure>
</center>

In figure [](#fig:duckiebottopview) the control input $\omega$ and the states $\vec{x}=\begin{bmatrix}d, & \varphi\end{bmatrix}^T$ of the plant are shown.

Finish the **controller-lqr.py** file. Test your controller using the command

    duckiebot $ make csii-ex3-lqr

and make sure to normalize your $R$ and $Q$ Matrices. Choose the corresponding weights and tune them until you achieve a satisfying behavior on the track.
