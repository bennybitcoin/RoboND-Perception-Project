#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    #sof = pcl_data.make_statistical_outlier_filter()
    #sof.set_mean_k(20)
    #sof.set_std_dev_mul_thresh(0.3)
    #filtered_pcl = fil.filter()
    
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


    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)


    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs: 
    print("here")
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects = []
    detected_objects_labels = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert Cluster to ROS from PCL
        ros_pcl_array = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_pcl_array, using_hsv=True)
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

    
    

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables 
    test_scene_num = Int32()
    object_name    = String()
    arm_name       = String()
    pick_pose      = Pose()
    place_pose     = Pose()
    
    request_params = []
    centroids = []
    
    test_scene_num.data = 1


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
        
        for detected_object in object_list:
            if detected_object.label == object_name.data:
        
                points_arr = ros_to_pcl(detected_object.cloud).to_array()
                centroids = np.mean(points_arr, axis=0)[:3]


                # TODO: Create 'pick_pose' for the object
                pick_pose.position.x = np.asscalar(centroids[0])
                pick_pose.position.y = np.asscalar(centroids[1])
                pick_pose.position.z = np.asscalar(centroids[2])
                break


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
        # not attempting pick and place
        #try:
        #    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
         #   resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

          #  print ("Response: ",resp.success)

        #except rospy.ServiceException, e:
        #    print "Service call failed: %s"%e


    # TODO: Output your request parameters into output yaml file
    yaml_file = "output_{}.yaml".format(test_scene_num.data)
    send_to_yaml(yaml_file, request_params)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous = True)


    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

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
