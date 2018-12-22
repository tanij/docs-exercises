# Exercise: Train an Object Detector {#exercise-object-detector status=draft}

Assigned: Orlando Marquez and Jon Plante

## Skills learned

- Label images from Duckietown logs to perform object detection
- Prepare data to train a neural network that performs object detection: YOLO 
- Train YOLO on labelled images
- Run the object detector on images or videos

## Motivation

YOLO is a real-time object detection system with a small footprint, if using tiny YOLO, which is a version of the YOLO architecture that has fewer convolutional layers. The size of the trained weights of tiny YOLO is less than 50 Mb.

 An object detection system can form the basis of a more complex pipeline, for instance, when doing SLAM. Below, we provide instructions on how to train tiny YOLO on 4 classes of objects:

1. Duckiebots
2. Duckies
3. Stop signs
4. Road signs

We hope to make it easier for anyone to train a Duckietown object detection system on more classes and with more data (to improve accuracy).

## Instructions

Clone the Github repository forked from YOLO's repository:

    laptop $ git clone https://github.com/marquezo/darknet

This repository contains the YOLO files, a dataset that we created and tools to train an object detection system.

### Creating a dataset
The repository contains a small dataset of 420 images `data_4_classes.tar.gz`. We provide tools to expand this dataset or create a new one with the classes of objects you want to detect.

First, if you are using the Duckietown logs, you will see that many of the images are blurry. Some of them are too blurry for anything useful to be learned. 


We also provide a script to label images:

    laptop $ python label_data.py

You need to execute `pip install easygui` if you don't have `easygui` installed.

### Preparing the dataset
At this stage, you should have a directory called `data` with two sub-directories `frames` and `labels`. From there we want to create the directories that we will use to train YOLO. We provide a script named `create_datasets.py` which will create the required directories:

    laptop $ python create_datasets.py 



### Training


### Testing


