#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.features import extract_features
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

    print 'hello'



    # TODO: Convert ROS msg to PCL data

    pcl_cloud = ros_to_pcl(pcl_msg)

    cloud_filtered = pcl_cloud

    # TODO: Statistical Outlier Filtering

    outlier_filter = cloud_filtered.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(5)

    # Set threshold scale factor
    x = 0.1

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()


    # TODO: Voxel Grid Downsampling

    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_filtered.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.01

    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()



    # TODO: PassThrough Filter

    # Create a PassThrough filter object. - Z
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()

    # Create a PassThrough filter object. - Y
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()




    # TODO: RANSAC Plane Segmentation

    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table
    max_distance = 0.001
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers

    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    # Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Statistical Outlier Remove
    outlier_filter = cloud_objects.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)

    # Set threshold scale factor
    x = 0.1

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_objects = outlier_filter.filter()

    pcl_test_pub.publish(pcl_to_ros(cloud_objects))

    # TODO: Euclidean Clustering

    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.04)
    ec.set_MinClusterSize(60)
    ec.set_MaxClusterSize(1500)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()



    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    cluster_msg = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_cluster_pub.publish(cluster_msg)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)

    detected_objects_labels = []
    detected_objects = []

    for j, indices in enumerate(cluster_indices):

        # Grab the points for the cluster
        cluster = cloud_objects.extract(indices, negative=False)
        cluster_msg = pcl_to_ros(cluster)

        # Compute the associated feature vector
        features = extract_features(cluster_msg, using_hsv=True)
        features = X_scaler.transform([features])

        # Make the prediction
        prediction = clf.predict(features)

        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[indices[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, j))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = cluster_msg
        detected_objects.append(do)

    # Publish the list of detected objects

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # get parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')



    # create dict label->centroid
    object_centroid = {}
    for obj in object_list:
        label = obj.label
        points_arr = ros_to_pcl(obj.cloud).to_array()
        centroid = np.mean(points_arr, axis=0)[:3]
        object_centroid[label] = centroid

    dropbox_position = {}
    for dp in dropbox_param:
        dropbox_position[dp['group']] = dp['position']

    # print 'object_centroid = ', object_centroid
    # print 'dropbox_position = ', dropbox_position

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map


    dict_list = []

    # TODO: Loop through the pick list
    for pick_item in object_list_param:
        pick_label = pick_item['name']
        pick_group = pick_item['group']

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        if not object_centroid.has_key(pick_label):
            continue

        centroid = object_centroid[pick_label]

        print pick_label, ' = ', centroid

        # Prepare param: test_scene_num
        test_scene_num = Int32()
        test_scene_num.data = scene_num

        # Prepare param: object_name
        object_name = String()
        object_name.data = pick_label

        # TODO: Assign the arm to be used for pick_place

        # Prepare param: which_arm
        which_arm = String()
        if pick_group == 'green':
            which_arm.data = 'right'
        else:
            which_arm.data = 'left'

        # Prepare param: pick_pose
        pick_pose = Pose()
        pick_pose.position.x = np.asscalar(centroid[0])
        pick_pose.position.y = np.asscalar(centroid[1])
        pick_pose.position.z = np.asscalar(centroid[2])

        # TODO: Create 'place_pose' for the object

        # Prepare param: place_pose
        place_pose = Pose()
        pp_position = dropbox_position[pick_group]
        place_pose.position.x = pp_position[0]
        place_pose.position.y = pp_position[1]
        place_pose.position.z = pp_position[2]


        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        yaml_dict = make_yaml_dict(test_scene_num, which_arm, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        '''
        # Wait for 'pick_place_routine' service to come up
        # try:
        rospy.wait_for_service('pick_place_routine', timeout=1)
        # except rospy.ROSException:
        #     print 'sevice busy !!! - BREAK'
        #     break


        try:

            # if picked.has_key(pick_label):
            #     continue

            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, which_arm, pick_pose, place_pose)

            print ("Response: ",resp.success)

            # if resp.success:
            #     break

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        '''



    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_' + str(scene_num) + '.yaml', dict_list)




if __name__ == '__main__':

    # picked = {}

    # TODO: ROS node initialization
    rospy.init_node('perception')

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_test_pub = rospy.Publisher('/pcl_test', pc2.PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', pc2.PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)

    # NOTE: Change this param in accordance to the world
    scene_num = 1

    # TODO: Load Model From disk
    model1 = pickle.load(open('model1_900_986.sav', 'rb'))
    model2 = pickle.load(open('model2_900_966.sav', 'rb'))
    model3 = pickle.load(open('model3_1500_960.sav', 'rb'))

    models = [model1, model2, model3]

    model = models[scene_num-1]

    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    X_scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
