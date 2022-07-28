# Multiclass Semantic Segmentation System

This system uses the [Catalyst](https://github.com/catalyst-team/catalyst) and [Segmentation Models PyTorch](https://github.com/qubvel/segmentation_models.pytorch) libraries. 

<!---
Add image once model is trained!

<p align="center">
  <img src="">
</p>

--->

## Folder Structure 

+ **multiclass_segmentation.py**: launches the multiclass segmentation rclpy node
+ **requirements.txt**: lists dependencies
+ **train.py**: implements, trains, and tests the model
+ **urc_multiunet.ipynb**: for training use with Google colab

### config

+ **multiclass_segmentation_params.yaml**: parameters for the multiclass segmentation node
+ **urc.yaml**: parameters related to training and testing the model

### data_utils

+ **data_loaders.py**: defines the Torch data loaders
+ **make_dataset.py**: converts folders of images and masks into .npy files
+ **segmentation_dataset.py**: defines the utilized dataset
+ **split_data.py**: splits images and masks into training and testing sets

### jupyter_notebook

+ **UNetWithEfficientNet.ipynb**: implements the training and testing of the neural network in a notebook

### train_utils

+ **get_args.py**: parses arguments for training
+ **helper_operations.py**: defines multiple operations used for loss and metric calculations
+ **save.py**: saves the image, mask, and predicted mask



## Build Instructions 

1. Download the dependencies in requirements.txt manually or using `pip install -r requirements.txt`
  
2. Obtain a dataset containing the images (PNGs) and masks (JSONs) [here](https://cloud.robojackets.org/apps/files/?dir=/RoboNav/Software/dataset/urc_dataset&fileid=356424)
**Skip steps #3 and #4 if using .npy files**

3. Run `python make_dataset.py -images /path_to_images(png) -masks /path_to_masks(json)`to generate .npy files. Example: `python make_dataset.py -images 'annotations/**/*.png' -masks 'annotations/**/*.json'`
   
4. Establish train_images.npy, train_masks.npy, test_images.npy, and test_masks.npy files with `python split_data.py -images '/path_to_images.npy' -masks '/path_to_masks.npy'` 
   
5. Run `python train.py -train_images '/path_to_train_images' -train_masks '/path_to_train_masks' -test_images '/path_to_test_images' -test_masks '/path_to_test_masks'` to train and test the neural network

## Visualize results
The training result can be visualized with TensorBoard using following commands.
`tensorboard --logdir=content`

## Train a model using Google Colab
You can alternatively use a google colab to train your model. Here is a brief instruction about setting up google colab environment.
1. First visit to [Google Colaboratory](https://colab.research.google.com/notebooks/intro.ipynb#recent=true) and upload the **urc_multiunet.ipynb**.
2. Download the required dataset [here](https://cloud.robojackets.org/apps/files/?dir=/RoboNav/Software/dataset/urc_dataset&fileid=356424). 
3. Follow the instruction on the **urc_multiunet.ipynb**


#### If using your own dataset perform the following steps:

1.  Upload all the images and associated masks to the shared drive. Each mask and its associated image should be in the same directory.

```
├── dataset1
│ ├── 00000000.jpg
│ ├── 00000000.json
│ ├── 00000001.jpg
│ ├── 00000001.json
```

2.  Open the [ConvertImages_SplitData.ipynb](https://colab.research.google.com/drive/15T2XQjBKMh92A3XlsfR-kRX-PFVN9rl4?usp=sharing) Google Colab Notebook and follow the steps to convert your dataset into the appropriate format and create train/test splits.
    
3.  Open the [urc_multiunet.ipynb](https://colab.research.google.com/drive/1bb9TRCNWBgV8-EiqzhjlSQM_yny9DXzt?usp=sharing) Google Colab Notebook and follow the steps to train the network. Make sure to adjust the config file in urc_perception/multiclass_segmentation/config/urc.yaml for desired training parameters and to specify model save location.
    - Note: Tensorboard may not show anything when launched for the first time so try pressing the refresh button on tensorboard in the top right of the screen.

## Archived notebook

If you like to run an old version of Jupyter Notebook, download **UNetWithEfficientNet.ipynb**, and run it in [Google Colaboratory](https://colab.research.google.com/notebooks/intro.ipynb#recent=true) or a local Jupyter environment. 