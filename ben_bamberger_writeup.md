## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  


### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.


--- Voxel Downsampling ---
The first step of Exercise 1 was to complete Voxel Grid Downsampling. The goal of Voxel Grid Downsampling is to reduce the data points necessary to represent the 3-D space. This allows for better computational efficiency. Below is the code I sued to implememnt my voxel grid filter.

```
vox = cloud.make_voxel_grid_filter()

LEAF_SIZE = 0.005


# Voxel Grid filter
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)
```

The provided PCL library had the function already created to enable the voxel grid downsampling. My only decision was to provide the leave size which determines the volumetric size of each averaged voxel. To achieve the results in the pricture below I used a leaf size of 0.005 meters(5mm). You can see though the resolution has decreased by the downsampling it is still very easy to differentiate the items and their locations.


--- Passthrough Filter ---
The Passthrough Filter was used to remove unnecssary data from the 3-D space. Essentially we are cutting down the point cloud to only include the areas of interest. This is done by simply choosing values of an axis to target. In our case we used the z-axis to trim our selection to just the objects on the tabletop. Below is the code I used to complete this:

```
# PassThrough filter
pt = cloud_filtered.make_passthrough_filter()
filter_axis = 'z'
pt.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
pt.set_filter_limits(axis_min, axis_max)

cloud_filtered = pt.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)
```
So as you can see, I simply selected to crop my point cloud to only the values which are 0.6m to 1.1m from the Z-plane. The resulting point cloud is shown in the image below:

--- RANSAC plane fitting ---
Our next step is now to remove the table from the point cloud data. To do this I utilized Random Sample Consesus or "RANSAC" plane fitting. This method helps us determine what points belong to individual objects in the scene. The RANSAC algorithm classifies points as either "inliers" (necessary data) versus "outliers" (unwanted data). I set a maximum distance value of 0.01 meters for the alogrithm to consider fitting the model. Again, we luckily have a RANSAC function built-in with the PCL library:

```
# RANSAC plane segmentation
# Create the segmentation object
seg = cloud_filtered.make_segmenter()

# Set the model you wish to fit 
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance 
# for segmenting the table
max_distance = 0.01
seg.set_distance_threshold(max_distance)


# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()
```

In the code below I save the inliers and the outliers as individual point clouds for analysis
```
# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)

# Save pcd for table
# pcl.save(cloud, filename)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)
```
Image of Extracted inliers:

```
# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative=True)

# Save pcd for tabletop objects
filename = 'extracted_outliers.pcd'
pcl.save(extracted_outliers, filename)
```
Image of Extracted outliers:

--- Statistical Outlier Filter ---
The final step of exercise 1 was to complete the statistical outlier filter. This will remove any unnecessary noise from the point cloud. After this is completed we will have a more clear picture of the focal objects in the space. There was no noise in the data for this exercise so it was not effective. It will however be required in the final project requirement.

```
# Much like the previous filters, we start by creating a filter object: 
outlier_filter = cloud_filtered.make_statistical_outlier_filter()

# Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(50)

# Set threshold scale factor
x = 1.0

# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
outlier_filter.set_std_dev_mul_thresh(x)

# Finally call the filter function for magic
cloud_filtered = outlier_filter.filter()
```


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



