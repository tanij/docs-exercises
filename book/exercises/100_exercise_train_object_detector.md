# Exercise: Train an object detector {#exercise-object-detector status=draft}

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

We also provide a script to label images: `label_data.py`. This scripts allows you to draw rectangles around an object in image and then label the class of that object. You can label multiple classes.

    laptop $ python label_data.py ![input_folder] ![output_folder]

You need to execute `pip install easygui` if you don't have `easygui` installed. Right now, it allows labelling up to 4 classes. 
The arguments are:

* `input_folder`: directory of images that you want to label
* `output_folder`: directory where labelled images will be moved to and where the corresponding label file will be saved

This script will show you all files with extension `jpg` or `jpeg`. For each image, you can draw a rectangle around the object you want to label. To finish labelling an image press control+c when on the picture, or enter 0 as the class of an object. This object will not be in the label file, and the program will go to next image. Classes start at 1 in the input dialog box, but in the label file it starts at 1. The label file is a txt file in YOLO format with the same name as the image file. To quit, just press control+c in the command prompt.

Finally, we provide a script to ensure that the labels work as expected: `check_annotation.py`:

    laptop $ python check_annotation.py ![image_path] ![label_path]

The arguments are:

* `image_path`: image that will be checked
* `label_path`: file that has labels in it

This script will show the image and the bounding boxes represented by the labels found in `label_path`. Classes with  different labels will have different colors. Press any key to close the window showing the image.

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

We provide a sample bash script that performs all the steps to prepare the data. It is called `prepdata`.


### Prepare the YOLO config files
Now that the datasets have been created, we need to create the YOLO configuration files. These are:

* File containing the class names (`classname_file`)
* File specifying where the training/validation sets are as well as where to save the weights during training (`data_file`)
* File specifying the neural network architecture to use (`architecture_file`)

To start the training, we need to call:

    laptop $ ./darknet detector train ![data_file] ![architecture_file] ![pretrained_weights]

The `data_file` is important as this specifies the data to use while training. It has the following lines:

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

These are the 4 classes of objects we are training on. An example label file is `duckiestuff/trainset/100_000151.txt` with a corresponding image file `duckiestuff/trainset/100_000151.jpg`. Note that they have to be in the same directory. The label for this image is specified as follows:

```
2 0.4 0.22 0.02 0.1
3 0.75 0.22 0.03 0.1
1 0.99 0.44 0.03 0.07
```

Each line of this file refers to an object. There are 5 elements in the YOLO label, each separated by a space:

1. Class ID (in the same order as in the file containing the class names). Class 0 is bot in our case, class 1 is duckie and so on
2. x coordinate for the center of the object in the image
3. y coordinate for the center of the object in the image
4. object width (normalized by the width of the image)
5. object height (normalized by the height of the image)

Lastly, there is the `architecture_file`, which specifies the number of convolutional layers to use and has to match the number of classes we are trying to detect. We simply copy `cfg/yolov3-tiny.cfg` and make the following modifications:

1. Change lines 127 and 171 to filters=27 as we have 4 classes. The formula is filters=(classes + 5)\*3
2. Change lines 135 and 177 to classes=4 as we are training 4 classes

### Training

For the training steps, refer to the following [Google Colab](https://drive.google.com/open?id=17pV9CtC8MFi38z1gz8CqE0gD6DthDNMV), which you can copy and modify. This Colab starts by cloning the same repository as above.

### Testing

Once we have trained weights, we can run inference on an image, as follows:

    laptop $ /darknet detector test ![data_file] ![architecture_file] ![weights_file] ![input_image] -thresh ![threshold]

The `weights_file` is the result of our training. The `threshold` parameter allows us to tell the predictor to only output bounding boxes when it is highly confident of its predictions. Here is an example of the bounding boxes predicted using a threshold of 0.7.

<img style="width:30em" src="yolo_sample.jpg"/>

We can also run inference on a video file, for example a video from the Duckietown logs. However, it needs to be run from a machine with a GPU; otherwise, the frame-per-second rate will be too low.

    laptop $ ./darknet detector demo ![data_file] ![architecture_file] ![weights_file] ![input_video]

