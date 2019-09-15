# Exercise: Discretize the System {#exercise-HW3_1 status=draft}

## Skills learned

* discretize a plant

## Discretization

 In this exercise, the Duckiebot SIMO system will be discretized. Here we will not only test, but also design the controller in discrete time which will make updating the weights easier once you test your controllers on the real system. The continuous time model of a Duckiebot is:

 $\dot{\vec{x}}=A\vec{x}+Bu=\begin{bmatrix}0 & v\\ 0 & 0\end{bmatrix}\vec{x}+\begin{bmatrix}0\\1\end{bmatrix}u$

 $\vec{y}=C\vec{x}=\begin{bmatrix}1 & 0\\ 0 & 1 \end{bmatrix}\vec{x}$

With state vector $\vec{x}=\begin{bmatrix}d, & \varphi\end{bmatrix}^T$ and input $u=\omega$. Notice, that the matrix C is an identity matrix, which means that the states are directly mapped to the outputs.

Discretize the system in terms of velocity $v$ and the sampling time $T_s$ using exact discretisation and test your discretization using the provided ([$\text{MATLAB}^\text{®}$-file](https://github.com/duckietown/docs-exercises/tree/master19/book/exercises/200_control_systems/additional_material)).

Note: If $A$ is not invertible, you have to use the formula $B_d = \left(\int^T_0 e^{A\tau}d\tau\right)B$.
