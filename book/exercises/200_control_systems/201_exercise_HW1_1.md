# Exercise: Prepare the environment {#exercise-HW1_1 status=draft}


## Skills learned

* getting started.
* use Duckietown as a platform.


## Prepare the environment.

This exercise gives you an introduction on how-to use the platform. This is done like this:


### Initial setup

Login into your computer, turn on your Duckiebot and make sure that your computer is connected to the same network as your Duckiebot.

In order to confirm that system is booted (it may take up to 2 minutes) open the web interface at [http://vehicle_name.local:9000](http://vehicle_name.local:9000) where `vehicle_name` is the name of your Duckiebot. If you encounter problems try to remove **.local** from the hostname [http://vehicle_name:9000](http://vehicle_name:9000).

Open two terminals and display them next to each other. We will use one (let us call this <img style="height:1em; width:1em" src="laptop.pdf"/>) local on the computer, and the other one (let us call that one <img style="height:1em; width:1em" src="duckiebot.pdf"/>) to connect to the Duckiebot. When we enter into a Docker container in order to execute some commands the Docker sign (<img style="height:1em; width:1em" src="docker.pdf"/>) will appear next to the terminal sign.

Clone the git repository in a folder **CSII** in your home directory.

    laptop $ git clone https://github.com/idsc-frazzoli/CS2_2019HWexercises.git ~/CSII/

### Connect and prepare

Connect to your Duckiebot  

    duckiebot $ cd ~/CSII/
    duckiebot $ make -i connect-![vehicle_name]  

Where `vehicle_name` is the hostname of Duckiebot. If done properly, you should see a logo of Duckie in the terminal.

Prepare the environment of the laptop

    laptop $ cd ~/CSII/
    laptop $ source prepare-CSII.sh
    laptop $ make copy-to-![vehicle_name]

Note: This command will erase previous work in CSII folder on Duckiebot!


### Get familiar with your Duckiebot:

On your Duckiebot, start the execution of exercise 1:

    duckiebot $ make csii-ex1-1

This will take about 30s.

In the meantime, start the virtual joystick on your computer:

    laptop $ dts duckiebot keyboard_control ![vehicle_name]

A new window will open. Whenever the window focus is on that window, you're able to steer your Duckiebot using the following keys:

  | KEY            | MOVEMENT                    |
  | -------------- | --------------------------- |
  | ARROWKEYS    	 | Steer your Duckiebot        |
  | <kbd>A</kbd> 	 | Steer your Duckiebot        |
  | <kbd>S</kbd> 	 | Stop lane followin        	 |
  | <kbd>Q</kbd> 	 | Quit the virtual joystick 	 |


As soon as both the virtual joystick and exercise 1 are running, let your Duckiebot follow the lanes by placing it into a road and hitting <kbd>A</kbd> while your window focus is on the virtual joystick. Be ready to press <kbd>S</kbd> in case your Duckiebot moves out of the lane or you need to stop it for some other reason.

Congratulations, your Duckiebot is now using a P-controller to stay inside the lanes!


###Tuning a P-controller

As you for sure have realized, this P-controller isn't a very good solution - the Duckiebot reacts very slow and it leaves the lanes after a curve. This happens due to the low gain of the controller used in controller-1.py from which follows a very long settling time. As a first step, we would like to increase the gain of the P-controller.

For this, first stop the execution of exercise 1 on your Duckiebot by pressing <kbd>Ctrl</kbd>-<kbd>C</kbd> inside the terminal <img style="height:1em; width:1em" src="duckiebot.pdf"/>.

Stop the execution of the virtual joystick by pressing <kbd>Q</kbd> while the window focus is on it (<img style="height:1em; width:1em" src="laptop.pdf"/>).

Note that you need to restart the virtual joystick every time you start the execution of an exercise.

Open Atom on your computer (Double click on Desktop icon), then File $\rightarrow$ Add project folder and choose CSII in home folder.

Navigate to **Exercises/HWExercise1** and edit the file named **controller-1.py**.

After editing, save the file (<kbd>Ctrl</kbd>-<kbd>S</kbd> in Atom) and navigate back to CSII folder in terminal to copy it on to Duckiebot.

    laptop $ cd ~/CSII/
    laptop $ make copy-to-![vehicle_name]

Note that this will overwrite the content of **CSII** folder at Duckiebot!  

In order to check that changed files are copied to Duckiebot open the file with **vim**.  

    duckiebot $ vim CSII/Exercises/HWExercise1/controller-1.py

To exit **vim** without changes press <kbd>Esc</kbd>, type **:q!** and press <kbd>Enter</kbd>. If you want to edit the file press <kbd>I</kbd>, you will notice that the keyword <kbd>Insert</kbd> will appear in the bottom of the terminal. To save changes and exit, first leave the <kbd>Insert</kbd> mode by typing <kbd>Esc</kbd> then type **:x** and press <kbd>Enter</kbd>.

**CSII** folder in the Duckiebot can be copied to your home folder.

    laptop $ cd ~/CSII/
    laptop $ make copy-from-![vehicle_name]

Note: This will overwrite the content of **CSII** folder in the laptop!

Also, note that again you need to be in **CSII** folder.

Get familiar with this file. Basically, the function **getControlOutput** is called every time after a new image was taken and the detection and estimation algorithms were executed. Then, it calculates what angular and linear speeds need to be given such that the bot reaches its equilibrium. The input and output variables are described inside the file.

Adjust the gain **self.k_P** such that the P-controller responds more heavily. Then, execute exercise 1b again and let your Duckiebot follow the lane (refer to point b). Notice the difference!

    laptop $ make copy-to-![vehicle_name]

Note: That copying files will always overwrite them! We recommend to work on your computer and and use this command to copy it to Duckiebot!
