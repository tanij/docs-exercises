# Exercise: Free space estimation with Stixel World

Assigned: Laurent Mandrile and David Abraham

## Skills learned

- What is free space estimation
- How to mathematically obtain Stixel World
- Getting a Stixel Representation based on a stereo image

## What is free space estimation

Free space is the physical space that is driveable and collision free around an agent.

## How to mathematically obtain Stixel World

Here is a very high level summary of how to obtain a stixel representation.

Stixel representation is a middle ground between pixel and object level representations.

A stixel is a rectangular “stick” of a given width defined by its 3D orientation relative to the camera. Its height is defined by the bounding box of the object it represents. And free space is represented by the area between stixel agglomerations.


<div figure-id="fig:example1" figure-class="flow-subfigures">
    <img figure-id="stixel obtaining flow" src='evolution.png'/>
</div>


### Building the stixel world

Stixel world construction starts with a stereo vision analysis to get a disparity map.

### Building the occupancy grid 

Afterwards, an occupancy grid is computed from the stereo data.
An occupancy grid is a 2D representation which models occupied pixels in an image of the environment.

### Background removal

A background subtraction is carried out All occupied cells behind the first maximum is marked as free.

### Free Space

Find the first obstacle when starting from the bottom Only the first obstacle is taken into account.

### Height Segmentation 

Height of objects is obtained by finding the optimal segmentation between the foreground and the background.
Afterwards, we compute the membership value of each point to a given object: The image is divided in columns. For each column a foreground object starts where the free space ends.
Then, the object continues column-wise and each point on the image votes for its membership to the foreground object
If it does not deviate more than a maximal distance from the expected disparity of the object based on the sgm analysis, the disparity votes positively. Otherwise, it votes negatively.

### Cost Image 

From the cost image we can extract the height of the obstacles. For each column, the height is where the cost is at its maximum.


<div figure-id="fig:example2" figure-class="flow-subfigures">
    <img figure-id="Cost function" src='cost.png'/>
</div>


### stixel extraction

With the free space and the height of the obstacles, we can finally create the stixels

## Running a live Stixel Representation demo based on stereo images

### Install dependencies

#### Opencv installation on ubuntu

First, make sure you have some dependencies for opencv itself:

```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

Next, create any directory for it to contain your opencv installation, cd into it and grab the source code:

```
cd /path/to/desired/opencv/install/location
git clone --branch 3.4 https://github.com/opencv/opencv.git
```

Then create a build directory and configure (don't forget to replace the path in the cmake command to the folder you downloaded the source code in):

```
cd ~/opencv
mkdir build
cd build
cmake -DOpenCV_DIR="/path/to/desired/opencv/install/location/opencv/build" ../
make -j7
```

### Download required code and data to run an example

Here's the link to download some sample data (stereo images) and the stixel-world demo : [link](https://drive.google.com/open?id=1Qmjo0ie79VV4dZ23e99a3eo7q_dGQRHe)

### Run the live demo

Uncompress the downloaded tarball to your preferred location and cd into it. To start the live demo:

```
./stixelworld goodWeather/2010-05-03_080828/images/img_c0_%09d.pgm goodWeather/2010-05-03_080828/images/img_c1_%09d.pgm ../camera.xml
```

Here's an example of what you should see:

![stixel-world example](stixel-world-demo.gif)
