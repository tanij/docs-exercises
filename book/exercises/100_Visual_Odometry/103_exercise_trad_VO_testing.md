# Exercise: Traditional Visual Odometry test {#exercise-traditional_VO-test status=ready}

<div class='requirements' markdown='1'>

Requires: You have finished [](#exercise-data-collect).

Result: Predicted poses of test data using Deep learning based VO.

</div>

## Preliminaries
First, make sure that you have a minimal environment, which includes:

1. Git
2. Catkin

Go to your workspace, clone and build [this repo](https://github.com/TienPoly/viso2.git)



    laptop $ cd ![your_ws]/src
    laptop $ git clone https://github.com/TienPoly/viso2.git
    laptop $ cd .. & catkin build



## Publish images
There are two options for this step.

### Online
Directly from camera of your Duckiebot as given [here](http://docs.duckietown.org/DT18/opmanual_duckiebot/out/read_camera_data.html)


### Offline
Running the commands below using the rosbag data like [a313.bag](https://drive.google.com/drive/folders/1Ym_YcDyScShnJ5A7JaPKI3zZ4jfEU161?usp=sharing)


    laptop $ roscore
    laptop $ rosbag play ![your_bag].bag



## Run Viso2

Launch Viso2 by running this command.


    laptop $ roslaunch viso2_ros mono.launch veh:=![vehicle_name]


If you use the example rosbag given above (`a313.bag`), the `vehicle_name` to be used is `a313`.


## Check your results
Once, Viso2 starts, you can use the ROS utilities `rqt_image_view` and `rqt_mulitplot` to view the result as in [](#fig:viso2_screen).


<div figure-id="fig:viso2_screen">
<img src="images/viso2_screen_new.png" style="width: 80%"/>
<figcaption>Viso2 example </figcaption>
</div>


You can checkout the Viso2 output of the given rosbag data (`a313.bag`) in this [video](https://youtu.be/TkJoXlgBYis).
