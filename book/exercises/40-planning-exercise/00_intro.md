# Agenda for Exercise session

1. AIDO submission

  1.1. Lane following task - description

  1.2. Agent template for aido1\_LF1

  1.3. Gym-duckietown - small demo (manual\_control.py, inputs and outputs)

2. Planning exercise

  2.1.Install needed software and dependencies

  2.2.Pose representation and operations tutorial
  
  2.3.Dynamics tutorial
3. Questions

## AIDO submission

[Lane following task - description](http://docs.duckietown.org/DT18/AIDO/out/lf.html)

All students should clone the following repo:

[https://github.com/duckietown/challenge-aido1\_LF1-template-random](https://github.com/duckietown/challenge-aido1_LF1-template-random)

Open `challenge-aido1\_LF1-template-random/solution.py` with your favorite text editor or IDE.

At the beginning we initialize the simulator environment and obtain the first observation:

    env = gym.make(gym\_environment)

    observation = env.reset()

Observation is basically an image. Then we process that image with a function f.

    action = f(observation)

In our case it&#39;s going to be a random number generator. We generate an array of two random uniform numbers from 0.3 to 0.4.

    action = np.random.uniform(0, 0.1, 2) + 0.3

At the end we pass it to the environment and execute the step in the simulator.

    observation, reward, done, info = env.step(action)

**Checkpoint:** Change the function to whatever you think will go as far as possible in the lane following task. Note that action = [vel\_left, vel\_right]     (2 minutes)

**Gym-duckietown** repo can be found here

[https://github.com/duckietown/gym-duckietown](https://github.com/duckietown/gym-duckietown)

(Optionally - no actions needed by students)

(Optionally) We will run manual\_control.py  to present you how the simulator works.

(Optionally) Clone it and run. Note that action here is not the same as before. Action = [linear\_velocity, angle]

**Checkpoint:** Open the `submission.yaml`  and change
`user-label : "Random execution" to "AMOD18-AIDO not that random execution"`

**Checkpoint:** Submit the challenge: `dts challenges submit`

For more details consult:

[http://docs.duckietown.org/DT18/AIDO/out/challenge\_aido1\_lf1\_template\_random.html](http://docs.duckietown.org/DT18/AIDO/out/challenge_aido1_lf1_template_random.html)

## Planning exercise - needed software and dependencies

First, install `duckietown-world` in a virtual environment.

Clone: [https://github.com/duckietown/duckietown-world](https://github.com/duckietown/duckietown-world)

Create a virtualenv with:

    $ virtualenv venv

Activate it with:

    $ source venv/bin/activate

Install dependencies and build the duckietown-world:

    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps

If there is a problem with requirements try alternative:

[https://github.com/lapandic/duckietown-world/blob/planning-exercise-dzenan/requirements.txt](https://github.com/lapandic/duckietown-world/blob/planning-exercise-dzenan/requirements.txt)

**Checkpoint:** Make sure all students have SciPy, PyGeometry and PyContracts

Open the terminal, activate the virtualenv and type:

    $ pip show SciPy PyGeometry PyContracts

### Install Jupyter

If you have Python 3 installed:

    $ python3 -m pip install --upgrade pip
    $ python3 -m pip install jupyter

If you have Python 2 installed:

    $ python -m pip install --upgrade pip
    $ python -m pip install jupyter

To run the notebook, run the following command:

    $ jupyter notebook

**Checkpoint:** Make sure all students can open a jupyter notebook

Now run:

    $ jupyter notebook exercise\_1.ipynb

### Tutorial notebooks

[https://github.com/lapandic/duckietown-world/tree/planning-exercise-dzenan/notebooks](https://github.com/lapandic/duckietown-world/tree/planning-exercise-dzenan/notebooks)


### References

LaValle's book

[http://planning.cs.uiuc.edu/](http://planning.cs.uiuc.edu/)

### Potential problems:

If it happens that OpenSSL is causing troubles just remove it.

    $ sudo apt remove python-openssl
