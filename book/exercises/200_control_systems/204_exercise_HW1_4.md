# Exercise: Testing the impact of a smaller sampling rate {#exercise-HW1_4 status=draft}

## Skills learned

* understand impacts of lower sampling rates.
* recognize issues associated to time delays.

## Basics {#basics}

Note: Assume that a Duckiebot has a sampling time of $T \approx 70ms$.

Our model of a Duckiebot only works on a specific range of our states $d,\varphi$: If those values grow too large, the camera looses the lines and the estimation of the output $y$ is not possible anymore, the system will destabilize. By increasing $k_s$ in **controller-4.py**, check how much we can reduce the sampling rate until the system destabilizes. Notice that since our controller is discrete and therefore we can only increase the sampling time T in discrete steps $k_s$ where $T_{new} = k_sT$. This is implemented in the lane controller node: that node is called every time a new measurement of the lane pose was made. In order to reduce sampling rate, we only handle every $k_s$-th measurement ( $(\#measurement \bmod k_s) \equiv 0)$ ), and throw away all the other measurements. As before, to modify the controller use:

    duckiebot $ make csii-edit-ex1-4

And execute it with

    duckiebot $ make csii-ex1-4

## Improve robustness

For this system, a way to improve robustness to lower sampling rates is to use smaller gains $k_P$, $k_I$. Try to edit those parameters such that the system is stable again with that $k_s$ which destabilized your system in [](#basics).
