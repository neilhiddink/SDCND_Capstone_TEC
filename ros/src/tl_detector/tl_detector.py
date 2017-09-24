#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import threading

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.image_lock = threading.RLock()
        self.lights = []

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

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

        # Sim runs slower than real life
        # Use this to control frame droping for traffic light classifier
        #sub6 = rospy.Timer(rospy.Duration(0.2), do_something)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()


        ###  Tensorflow takes a while to load up,
        # This helps prevent issues while it's loading

        self.light_classifier_is_ready = False

        def light_warm_up_guard():
            self.light_classifier_is_ready = True
            t.cancel()

        print("Loading light classifier (5 second timer)")
        t = threading.Timer(5.0, light_warm_up_guard)
        t.daemon = True
        t.start()

        self.light_classifier = TLClassifier()
        print("Created traffic light classifier")
        

        # Deep net traffic light publisher (output of neural network)
        self.pub1_deep_net_out = rospy.Publisher('/deep_net_out', Image, queue_size=1)
        self.TrafficLightState = rospy.Publisher('/traffic_light_state', Int32, queue_size=1)

        self.listener = tf.TransformListener()
        print("Listening")

        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.image_lock.acquire(True):
            
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
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

            self.image_lock.release()


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement here or somewhere else?


        return 0

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        state = TrafficLight.UNKNOWN   # Default state

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        if self.light_classifier_is_ready is True:

            state = self.light_classifier.get_classification(cv_image)

            self.pub1_deep_net_out.publish(self.bridge.cv2_to_imgmsg(
                    self.light_classifier.image_np_deep, "rgb8"))

        
        #### TODO, nearest point?
        # Or should we return distance

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        
        self.TrafficLightState.publish(Int32(state)) 

        return -1, state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
