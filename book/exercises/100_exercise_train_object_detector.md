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
We include a small dataset of 420 images `data_4_classes.tar.gz` and provide tools to expand this dataset or create a new one with the classes of objects you want to detect.

First, if you are using the Duckietown logs, you will see that many of the images are blurry. Some of them are too blurry for anything useful to be learned. The script `detect_blurry_img.py` uses a Laplacien filter to determine if an image is blurry.

    laptop $ python detect_blurry_img.py ![input_folder] ![blurry_folder] ![non_blurry_folder] ![threshold]

The arguments are:

* `input_folder`: directory of all images you want to classify as blurry. We grab all files ending with `.jpg` or `.jpeg`
* `blurry_folder`: directory where blurry images will be copied to
* `non_blurry_folder`: directory where non blurry images will be copied to
* `threshold`: default is 200, the higher the threshold, the stricter the classifier is on blurry images

After executing this script the directory `non_blurry_folder` will contain the non-blurry images that we can train on.

We also provide a script to label images:

    laptop $ python label_data.py

You need to execute `pip install easygui` if you don't have `easygui` installed.

Finally, we include a script to verify the labels that were created:

    laptop $ python 

### Preparing the dataset
At this stage, you should have a directory called `data` with two sub-directories `frames` and `labels`. From there we want to create the directories that we will use to train YOLO. We provide a script named `create_datasets.py` which will create the required directories:

    laptop $ python create_datasets.py ![raw_data_folder] ![data_folder] ![percentage_training]

The arguments are:

* `raw_data_folder`: directory containing two sub-directories `frames` and `labels`, the former containing the images (`.jpg` files) and the latter has the labels (`.txt` files)
* `data_folder`: directory where we want to create three directories `trainset`, `validset` and `testset`
* `percentage_training`: how much of the data to use for training, between 80 and 90. The validation set and the test set will contain the rest of the examples, equally divided.

After execution, you need to create two other files that will be required during training

    laptop $ ls ![data_folder]/trainset/*.jpg > ![data_folder]/train.txt
    laptop $ ls ![data_folder]/validset/*.jpg > ![data_folder]/valid.txt

### Training
Now that the datasets have been created, we need to create the YOLO configuration files. These are:

* File containing the class names (`classname_file`)
* File specifying where the training/validation sets are as well as where to save the weights during training (`data_file`)
* File specifying the neural network architecture to use (`architecture_file`)

To start the training, we need to call:

    laptop $ ./darknet detector train ![data_file] ![architecture_file] ![pretrained_weights]

The `data_file` is important as this specifies the data to use while training. It looks like:

```
train  = duckiestuff/train.txt
valid  = duckiestuff/valid.txt
names = data/duckie-multi.names
backup = duckie_backup
```

You can look at the repository to get an idea of what files `train.txt` and `valid.txt` look like: lists of image paths. The file referenced by `names` is the `classname_file`, which in our case looks like the following since we are training on 4 classes:
```
bot
duckie
stop_sign
road_sign
```


### Testing


