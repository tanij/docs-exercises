# Exercise: Deep learning based VO test {#exercise-deep-vo-test status=beta}

<div class='requirements' markdown='1'>

Requires: You have finished [](#exercise-data-collect).

Result: Predicted poses of test data using Deep learning based VO.

</div>


## Adding the datasets to your Google drive
In order to run the DeepVO experiments using the Colab notebook, the data is required to added to your Google Drive. If you don't have a Vicon system to collect data yourself, you can use our datasets in this exercise.


Go to this [link](https://drive.google.com/drive/folders/1hecjZFEk9RHTdPx6eMawwU2jwGtPr_V3?usp=sharing) and click on "Add to my Drive".


We collected data as we described in [](#exercise-data-collect) and you can run DeepVO on the two datasets given here corressponding to different shapes of the trajectories :

* Two Loops  

* Eight


If you use dataset(s) other than the ones used here in the examples, you can use this [Colab notebook](https://colab.research.google.com/drive/1CKTIO4q_IXWS6-PJhLMROhqapQFG92fj) to compute the mean and standard deviation of the data for normalization.

You can checkout more details about the DeepVO implementation [here](https://drive.google.com/drive/folders/1m67FGZ0J3vrX486l492LsnaMlSLFH-WA?usp=sharing)

## Two Loops
You can follow the intruction on this [Colab notebook](https://colab.research.google.com/drive/1AVWgpkVtzAAgbuO01maq3e10fQ2esF05) and the expected results are documented [here](https://drive.google.com/drive/folders/1aIoT-NV3uc3e-PxAvUvvrQ7aJLlC1qhx).

<figure class="flow-subfigures">  
    <figcaption>Example of DeepVO - 2 Loops</figcaption>
    <figure>
        <img style='width:15em'
        src='images/loops4_truth.png'/>
        <figcaption>Ground truth</figcaption>
    </figure>
    <figure>  
        <img style='width:14em' src='images/loops4_DeepVO.png'/>
        <figcaption>Deep learning based VO</figcaption>
    </figure>    
</figure>


## Eight
You can follow the intruction on this [Colab notebook](https://colab.research.google.com/drive/1Z8wg7M-vEDh1-ZWqALco8Bj3Oi1Hz5O_) and the expected results are documented [here](https://drive.google.com/drive/folders/1aIoT-NV3uc3e-PxAvUvvrQ7aJLlC1qhx).  

<figure class="flow-subfigures">  
    <figcaption>Example of DeepVO - Eight</figcaption>
    <figure>
        <img style='width:14em'
        src='images/eight_truth.png'/>
        <figcaption>Ground truth</figcaption>
    </figure>
    <figure>  
        <img style='width:14em' src='images/eight_DeepVO.png'/>
        <figcaption>Deep learning based VO</figcaption>
    </figure>    
</figure>


With the neural networks here, you can run more tests using other data and changing the hyperparameters to obtain even better results, especially the "Eight" trajectory.
