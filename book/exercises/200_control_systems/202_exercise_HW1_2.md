# Exercise: Finding parameters implementing a discrete PI-controller {#exercise-HW1_2 status=draft}

## Skills learned

* discretize and implement a PI-controller.
* recognize implementation issues associated to integral controllers.


## Mathematical Basics

In order to improve the behavior of our controller from exercise 1 and eliminate the steady state error (aka the offset from the center of the lane), we would like to design a PI-controller and implement it in **controller-2.py** such that the Duckiebot has a more acceptable behavior.

<center>
<figure>
    <img figure-id="fig:duckiebot_topview" figure-caption="Lane Pose of the Duckiebot." style="width: 50%; height: auto;" src="duckiebot_topview.pdf"/>
</figure>
</center>

The continious-time model of a Duckiebot with the states $\vec{x} = [d \enspace \varphi]^T$, input $u = \omega$ and output $y$ with the states defined as in Fig. [](#fig:duckiebot_topview) looks like this:

$\dot{\vec{x}} = A\vec{x} + Bu = \begin{bmatrix} 0 & v \\ 0 & 0 \end{bmatrix}\vec{x} + \begin{bmatrix} 0 \\ 1 \end{bmatrix}u$

$\vec{y} = C\vec{x} =[6 \enspace 1]\vec{x}$

With transfer function $P(s) = \frac{s + 6v}{s^2}$.

Let $e = r - y$. A PI-controller for this system looks like

$u(t) = k(e(t) + \frac{1}{T_I}\int_{0}^{t}e(\tau) d\tau) := k_P e(t) + k_I\int_{0}^{t}e(\tau) d\tau$

in time domain and ...

$C(s) = k(1 + \frac{1}{T_Is}) := k_P + k_I\frac{1}{s}$ in frequency domain, where $U(s) = C(s)E(s) = C(s)(R(s) - Y(s))$.

## P-Controller

We are assuming that $v = 0.22 $ $ m/s$. By using  a tool of your choice (for example [https://goo.gl/EA8maj](https://julien.li/submit/bodeplot/)), try to find $k_P$ such that $L(s) = P(s)C(s)$ has a crossover frequency of approximately $4.2$ $ rad/s$.

## I-Controller

Try to find $k_I$ such that $L(s)$ has a gain margin of approximately $-25.6dB \approx 0.053$ (this refers to a gain of the controller which is about 19 times higher than the critical minimal gain that is needed for stability).

## Discretization

In order to implement this controller, we need to discretize it. There are several ways to do this, including the one presented in lecture 2. Let $u = C_P(t) + C_I(t)$. A popular method for a PI-controller found in *Feedback Systems from Aström and Murray* on page 311 *Computer implementation* is:

$C_P(t) = k_pe(t) \longrightarrow C_P(k) = k_pe(k)$

$C_I(t) = k_I\int_{0}^{t}e(\tau) d\tau \longrightarrow C_I(k+1) = C_I(k) + Tk_Ie(k)$
 where T is the time difference between two steps.

This method was used to implement a PI-controller in **controller-2.py**. Use your controller gains $k_P$ and $k_I$ in **controller-2.py** and see how it performs. Edit the file as in exercise 1 or directly in \duckiebot with

    duckiebot $ make csii-edit-ex1-2

Execute it with

    duckiebot $ make csii-ex1-2

Then start the keyboard control in terminal as in exercise [](#exercise-HW1_1).
