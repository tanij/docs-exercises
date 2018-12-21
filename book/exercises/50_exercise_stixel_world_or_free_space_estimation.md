# Exercise: Free space estimation with Stixel World

Assigned: Laurent Mandrile and David Abraham

## Skills learned

- What is free space estimation
- How to mathematically obtain Stixel World
- Getting a Stixel Representation based on a stereo image

## What is free space estimation

Free space is the physical space that is driveable and collision free around an agent

## How to mathematically obtain Stixel World

Stixel representation is a middle ground between pixel and object level representations

A stixel is a rectangular “stick” of a given width defined by its 3D orientation relative to the camera. Its height is defined by the bounding box of the object it represents. And free space is represented by the area between stixel agglomerations

### Building the stixel world

Stixel world construction starts with a stereo vision analysis to get a disparity map

### Building the occupancy grid 

Afterwards, an occupancy grid is computed from the stereo data
An occupancy grid is a 2D representation which models occupied pixels in an image of the environment

### Background removal

A background subtraction is carried out All occupied cells behind the first maximum is marked as free

### Free Space

Find the first obstacle when starting from the bottom Only the first obstacle is taken into account

### Height Segmentation 

Height of objects is obtained by finding the optimal segmentation between the foreground and the background
Afterwards, we compute the membership value of each point to a given object: The image is divided in columns. For each column a foreground object starts where the free space ends
Then, the object continues column-wise and each point on the image votes for its membership to the foreground object
If it does not deviate more than a maximal distance from the expected disparity of the object based on the sgm analysis, the disparity votes positively. Otherwise, it votes negatively.

### Cost Image 

From the cost image we can extract the height of the obstacles. For each column, the height is where the cost is at its maximum

### stixel extraction

With the free space and the height of the obstacles, we can finally create the stixels

## Getting a Stixel Representation based on a stereo image

