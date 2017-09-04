## Writeup for Perception Pick & Place Project

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

All implementation of the perception pipeline is in a file `pr2_robot/scripts/perception.py` so all references to the code point to this file.

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

This step consists of:

1) Statistical outlier filtering to clear noise. Number of points was set to 5 in order to make it quicker which also works reasonable well.

![statistical-outlier](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


2) Voxel Grid Downsampling to reduce the size of point cloud.

![voxel-downsampling](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


3) Passthrough filter is selecting the table plane with the objects.

![passthrough-z](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


4) RANSAC plane segmentation finds points of the table so we can select only points related to the objects.

![ransac-objects-selected](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


Then I also applied additional statistical outlier filter to remove the left over noise points.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

This step consists of:

5) Euclidean Clustering that selects the clusters of the objects

![eucledian-clusters](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

From cluster points I extract feature vector as `color_histogram` (from HSV channels) and `normal_histogram` split by 12 bins. Number of samples per class between 900-1500 worked well and for all tested worlds accuracy is higher than *96%*.

Model files located at `pr2_robot/results/model{1-3}.sav`.

Perplexity matrix for `world1`:

![World1-perplexity](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


Perplexity matrix for `world2`:

![World2-perplexity](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


Object recognition for `World 1`:

![World1-recognition](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


Object recognition for `World 2`:

![World2-recognition](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)


Object recognition for `World 3`:

![World3-recognition](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

NOTE: On `World 3` the object `glue` is occluded so it wasn't selected and recognized correctly. But considering the sequential pick and place operation it will become fully visible after `book` will be place into the box.

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

Object recognition for `test*.world` is shown above, see pictures.

Than I've created the messages and saved them to the `output_*.yaml` files as was shown in the lessons.

See output files in `pr2_robot/results/output_*.yaml`.

## Discussions

I've encountered couple of problems that need to be
solved for further improvements:

1) Statistical outlier filter is slow when performed on a full size point cloud. I've tried to apply it after voxel downsampling but without good results.

2) `World 3` has a challenging case when a `glue` occluded by `book` and wasn't clustered and recognized correctly.

3) Grasping is often broken. End effector positioning is good, down movement is good, grasping is not tight enough so object is just staying on the table.

4) Movement planning should include collisions with other objects.

5) Positioning of the end-effector above left and right box is not always presize. Seems like it's not accounting for the size of the holding object so `biscuits` not always correctly position into the box.

6) Movement command should be send eventually and account for environment changes due to the long pick and place execution. Some object can be bumped and moved or after picking one object we will be able to see the others. As in case of a `glue` and a `book`.
