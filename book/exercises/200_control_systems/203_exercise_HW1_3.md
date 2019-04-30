# Exercise: Effect of an anti-windup logic {#exercise-HW1_3 status=draft}

## Skills learned

* discretize and implement a PI-controller.
* recognize implementation issues associated to integral controllers.


## Basics

An integral part in the controller comes with a drawback in a real system: Due to the fact that the motors on a Duckiebot can only run up to a specific speed, we are not able to perform unbounded high inputs demanded by the controller. Imagine a Duckiebot with a very aggressive PI-controller from [](#exercise-HW1_2) which starts at a large offset from the equilibrium: The controller immediately demands a very high input $u$. Due to motor saturation, we can not create such a high input and it will take way longer to reach the equilibrium than without saturation of the motors. As soon as we reach the equilibrium, the integral will be extremely large and therefore the bot starts to overshoot up to the point when the controller demands an opposite signed input to the system. This overshoot will be significantly larger than the initial offset. This procedure will repeat, the overshoot will grow larger with every iteration and could potentially destabilize the system. It is referred to as integrator windup.

In order to overcome this effect, the implementation of an anti-windup logic resets the integral if motor saturation is achieved.

<center>
<figure>
    <img figure-id="fig:anti_reset_windup_ex" figure-caption="A PI-controller with an anti-windup logic implemented, Feedback Systems from Aström and Murray on page 308." style="width: 75%; height: auto;" src="PI-with-anti-windup.png"/>
</figure>
</center>

In [](#fig:anti_reset_windup_ex), you can see an implementation of an anti-windup logic for a PI-controller. $k_t$ determines how fast the integral is reset and is normally chosen in the order of $k_I$.

## First anti-windup experience

*controller-3.py*, we've implemented a PI-controller with the anti-windup logic from Figure [](#fig:anti_reset_windup_ex). The function $sat(self, u)$ tries to mimic the motor saturation of a Duckiebot. For this exercise, we're simulating a saturation at $u_{sat} = 2rad/s$. Note that whenever you execute exercise 3, this saturation is simulated in the background.

As a first step, test the performance of the Duckiebot with the anti-windup term turned off: $k_t = 0$. You will see that the performance is poor after curves. If you increase the integral gain $k_I$, you are even able to destabilize the system! Edit the file with

    duckiebot $ make csii-edit-ex1-3

Execute it with

    duckiebot $ make csii-ex1-3

## Anti-windup tuning

In order to avoid destabilization and improve the performance of the system, set $k_t$ to roughly the same value as $k_I$. Note the difference! *Optional:* With different values of $k_P$ and $k_I$, one could improve the behavior even more.

As you may have found out, for very aggressive controllers with an integral part and systems which saturate for relatively low inputs, the use of an anti-windup logic becomes necessary. In our case, an anti-windup logic is only needed if we introduce a limitation to the angular velocity $\omega$ - for example to simulate a real car (minimal turning radius).
