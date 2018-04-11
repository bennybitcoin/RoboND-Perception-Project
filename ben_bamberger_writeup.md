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
The first step of Exercise 1 was to complete Voxel Grid Downsampling. The goal of Voxel Grid Downsampling is to reduce the data points necessary to represent the 3-D space. This allows for better computational efficiency. Below is the code I sued to implement my voxel grid filter.

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

For this exercise we first went through the same steps required for Exercise 1 shown here:

```
# TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    
    # Voxel Grid filter
    vox = pcl_data.make_voxel_grid_filter()

    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    cloud_filtered = vox.filter()
    

    # PassThrough filter
    pt = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    pt.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    pt.set_filter_limits(axis_min, axis_max)

    cloud_filtered = pt.filter()
    

    # RANSAC plane segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.034 #increased from 0.01 because I was recognizing the front of the table as an object
    seg.set_distance_threshold(max_distance)


    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    # Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
```

--- Euclidian Clustering ---
After completing the filtering and RANSAC the next step is to preform Euclidean Clustering. First I converted XYZRGB to strictly XYZ point cloud data. Then I constructed a k-d tree from the cloud_objects point cloud. 

```
 
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    
    tree = white_cloud.make_kdtree()
```
Once that was completed I could complete the cluster extraction:
```

    ec = white_cloud.make_EuclideanClusterExtraction()

    ec.set_ClusterTolerance(0.01) #0.01
    ec.set_MinClusterSize(100) #25
    ec.set_MaxClusterSize(25000) #10000

    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract()
    #print(cluster_indices) #checked my parameters above for clustering
```


--- Cluster Mask Point Cloud: Visualization ---

Using this cluseter data, I created ROS messages and published them to a ROS topic to view my PCL data live in RViz. To start I assigned each separate cluster a unique color in order to differentiate them:

```
 # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2], rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    
```
Then I converted the pcl data to a Ros message using the built-in funtions. Finally I published the ROS messages to 3 topics: 1) pcl_objects - objects on table, 2) pcl_table - table itself 3) pcl_cluster - clustered objects
```
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)


    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
```
See screenshots from my RViz output below:

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
In this exercise I created an SVM classifier to recognize object types in my point cloud scene. The first step to creating the classifier is to create a training dataset. We do this by running the "capture_features.py" script to randomly generate our 7 objects in different 3-D orientations, this data was then saved to "training_set.sav" in the catkin workspace. Here is a screenshot from that runnning:


Next, we use this training data to train our SVM. We do so by simply running the "train_svm.py" script included with the Exercise. The initial confusion matrices were not optimal as we were not computing histograms. Here is the code I used to create the histograms from "/sensor_stick/src/sensor_stick/features.py":

```
 # TODO: Compute histograms
    r_hist = np.histogram(channel_1_vals, bins=32, range=(0, 256))
    g_hist = np.histogram(channel_2_vals, bins=32, range=(0, 256))
    b_hist = np.histogram(channel_3_vals, bins=32, range=(0, 256))

    # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((r_hist[0], g_hist[0], b_hist[0])).astype(np.float64)

    norm_features = hist_features / np.sum(hist_features)

    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    #normed_features = np.random.random(96) 
    return norm_features 


def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    x_hist = np.histogram(norm_x_vals, bins=32, range=(0, 256))
    y_hist = np.histogram(norm_y_vals, bins=32, range=(0, 256))
    z_hist = np.histogram(norm_z_vals, bins=32, range=(0, 256))

    # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((x_hist[0], y_hist[0], z_hist[0])).astype(np.float64)

    norm_features = hist_features / np.sum(hist_features)

    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    #normed_features = np.random.random(96)

    return norm_features
```
To further improve my classifier I increased the object spawn iterations from 5 to 9. My results for my improved confusion matrix is shown below:


The final step was to complete my object recognition on the love RViz data. Here is the Object Recoginition for loop that is run on each segmented cluster: (used code from previous exersizes to get the clusters)
```
detected_objects = []
    detected_objects_labels = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert Cluster to ROS from PCL
        ros_pcl_array = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_pcl_array, using_hsv=False)
        normals = get_normals(ros_pcl_array)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4
        object_markers_pub.publish(make_label(label, label_pos, index)) 
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_pcl_array
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    #print(detected_objects)
    
    if detected_objects:
        # Publish the list of detected objects
        detected_objects_pub.publish(detected_objects)
```

And finally I created new ROS publishers to broadcast my object recognition results to RViz:

```
if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous = True)


    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
```

My results in RViz for my final object recognition is shown in the screenshot below. I was able to successfully recognize all the objects with my model:

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then readsnequest output them to `.yaml` format.

--- Use filtering and RANSAC plane fitting ---

My first step was to apply the same filtering steps I used in thre previous exercises. (Voxel Grid to decrease data size and PassThrough Filter to look just at objects on the plane of the table)

```
# Voxel Grid filter
    vox = pcl_data.make_voxel_grid_filter()

    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    cloud_filtered = vox.filter()
    

    # PassThrough filter
    pt = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    pt.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    pt.set_filter_limits(axis_min, axis_max)

    cloud_filtered = pt.filter()
```


Then I completed the standard RANSAC plane segmentation to separate the objects from the table:

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
    max_distance = 0.034 #increased from 0.01 because I was recognizing the front of the table as an object
    seg.set_distance_threshold(max_distance)


    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    # Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
```

--- Apply Euclidean clustering ---
After filtering and RANSAC steps I am able to create k-d tree and perform Euclidean Clustering to separate the individual objects: (Notice I set the Cluster tolerance to 2x my Voxel Grid Filter LEAF_SIZE for best results, and I also increase the cluster size)

```
# TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()

    ec.set_ClusterTolerance(0.01) #0.01
    ec.set_MinClusterSize(100) #25
    ec.set_MaxClusterSize(25000) #10000

    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract()
    #print(cluster_indices) #checked my parameters above for clustering
```

--- Perform object recognition ---
Using the same for loop from Exercise 3 I was able to complete the object recognition for each cluster I separated in the Euclidean Clustering step.

```
# Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects = []
    detected_objects_labels = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert Cluster to ROS from PCL
        ros_pcl_array = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_pcl_array, using_hsv=False)
        normals = get_normals(ros_pcl_array)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4
        object_markers_pub.publish(make_label(label, label_pos, index)) 
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_pcl_array
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    #print(detected_objects)
    
    if detected_objects:
        # Publish the list of detected objects
        detected_objects_pub.publish(detected_objects)
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass
    else:
        ros.loginfo("No objects detected")
```

--- Calculate the centroid ---
Next I took my "detected_objects" and sent them to the pr2_mover function. There I calcualted the centroid of the clusters and broadcasted a message so that the pr2 simulator could pick the object.
```
def pr2_mover(object_list):

    # TODO: Initialize variables
    num_scene = rospy.get_param('/test_scene_num')   
    test_scene_num = Int32()
    object_name    = String()
    arm_name       = String()
    pick_pose      = Pose()
    place_pose     = Pose()
    
    request_params = []
    
    test_scene_num.data = num_scene


    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    print(object_list_param)
    dropbox = rospy.get_param('/dropbox')

   

    # Check consistency of detected objects list
    # if not len(detected_objects) == len(object_list_param):
    #     rospy.loginfo("List of detected objects does not match pick list.")
    #     return


    # TODO: Parse parameters into individual variables
    red_dropbox = dropbox[0]['position']
    green_dropbox = dropbox[1]['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for obj in object_list_param:

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        object_name.data = obj['name']
        points_arr = ros_to_pcl(obj.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

        
        # TODO: Create 'pick_pose' for the object
        pick_pose.position.x = np.asscalar(centroid[0])
        pick_pose.position.y = np.asscalar(centroid[1])
        pick_pose.position.z = np.asscalar(centroid[2])


        # TODO: Assign the arm to be used for pick_place
        if obj['group'] == 'red':
            arm_name.data = 'left'
            place_pose.position.x = red_dropbox[0]
            place_pose.position.y = red_dropbox[1]
            place_pose.position.z = red_dropbox[2]
        elif obj['group'] == 'green':
            arm_name.data = 'right'
            place_pose.position.x = green_dropbox[0]
            place_pose.position.y = green_dropbox[1]
            place_pose.position.z = green_dropbox[2]

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        request_params.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
```
--- Create ROS messages  and write these messages out to .yaml files ---
Finally I simply exported my ROS output request parameters into .yaml files:

    # TODO: Output your request parameters into output yaml file
    yaml_file = "output_{}.yaml".format(num_scene)
    send_to_yaml(yaml_file, request_params)

For future improvements I would like to incorporate steps in order to create a collision map of the objects as well as the sides of the table.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)




