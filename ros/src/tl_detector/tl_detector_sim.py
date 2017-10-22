#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import tensorflow as tflow
import numpy as np
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 1

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb,queue_size=1,buff_size=14400000)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.image_count = 0

        MODEL_NAME = 'traffic-light-sim-graph'
        PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

        self.detection_graph = tflow.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tflow.GraphDef()
            with tflow.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tflow.import_graph_def(od_graph_def, name='')

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1

            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
        if state == TrafficLight.GREEN :
           rospy.loginfo("---- GREEN ----")
           self.upcoming_red_light_pub.publish(Int32(-1))

    def load_image_into_numpy_array(img):
        (im_width, im_height) = img.size
        return np.array(img.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

    def get_euclidean_distance(self, pos1, pos2):
        return math.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2  + (pos1.z-pos2.z)**2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        min_dist = 999999
        min_ind = 0
        ind = 0
        car_position = pose.position

        # Calcuate distance between car and waypoint
        for waypoint in self.waypoints:
            waypoint_position = waypoint.pose.pose.position
            dist = self.get_euclidean_distance(car_position, waypoint_position)

            # Store the index of waypoint which is nearest to the car
            if dist < min_dist:
                min_dist = dist
                min_ind = ind

            ind += 1

        return min_ind

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.image_count = self.image_count + 1
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        height, width ,channels = cv_image.shape

        #x, y = self.project_to_image_plane(light.pose.pose.position)

        #file_name = "/home/shyam/Work/SDCNDP/Project13/Vidyut-CarND-Capstone/ros/images" + str(self.image_count) + ".jpg"
        #log_file = open("/home/shyam/Work/SDCNDP/Project13/Vidyut-CarND-Capstone/ros/images/log.txt", 'a')

        #TODO use light location to zoom in on traffic light in image
        if height != 600 or width !=800:
            cv_image = cv2.resize(cv_image, (800,600), interpolation=cv2.INTER_AREA)

        #cv2.imwrite(file_name, cv_image)
        pred_state = TrafficLight.UNKNOWN

        #print("Tensorflow version " + tflow.__version__)

        with self.detection_graph.as_default():
            with tflow.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                #image_np = load_image_into_numpy_array(cv_image)
                image_np_expanded = np.expand_dims(cv_image, axis=0)
                (boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections], feed_dict={image_tensor: image_np_expanded})

        class_id = []
        for i in range(len(scores[0])):
            if scores[0][i] > 0.5:
                class_id.append(classes[0][i])

        if (class_id):
            class_val = np.argmax(np.bincount(np.array(class_id, dtype=int)))
            if class_val == 1:
                pred_state = TrafficLight.RED
            elif class_val == 2:
                pred_state = TrafficLight.YELLOW
            elif class_val == 3:
                pred_state = TrafficLight.GREEN
                
        print("pred_state {}".format(pred_state))
        print("ground_truth {}".format(light.state))
        

        #log_file.write(file_name + " pred = " + str(pred_state) + " truth = " + str(light.state) + "\n")
        #log_file.close()

        #Get classification
        #return self.light_classifier.get_classification(cv_image)

        return pred_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        car_wp = 0

        if(self.pose):
            car_wp = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        stop_line_pose = Pose()
        stop_line_wp_list = []
        for i in range(len(stop_line_positions)):
            stop_line_pose.position.x = stop_line_positions[i][0]
            stop_line_pose.position.y = stop_line_positions[i][1]
            stop_line_pose.position.z = 0
            stop_line_wp = self.get_closest_waypoint(stop_line_pose)
            stop_line_wp_list.append(stop_line_wp)

        light = None

        # Get nearest waypoint to Traffic light stop line
        if(self.lights):
            min_dist = 99999
            light_wp = -1
            for i in range(len(stop_line_wp_list)):
                dist_car_light = stop_line_wp_list[i] - car_wp
                if dist_car_light > 0:
                    if dist_car_light < 150:
                        min_dist = dist_car_light
                        light_wp = stop_line_wp_list[i]
                        light = self.lights[i]

        #rospy.loginfo("--- Car wp %s",car_wp)

        if light:
            state = self.get_light_state(light)
            #rospy.loginfo("--- Light wp %s, state %s",light_wp, state)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
