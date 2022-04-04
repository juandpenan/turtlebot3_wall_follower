import rospy
import actionlib

from turtlebot3_wall_follower import controller

from turtlebot3_wall_follower.msg import StartAction
from turtlebot3_wall_follower.msg import StartResult
from turtlebot3_wall_follower.msg import StartFeedback

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class BurgerServer:

    def __init__(self):
        
        self.action_server = actionlib.SimpleActionServer('burger_commands',
                StartAction, execute_cb=self.on_goal, 
				auto_start=False)
        self.action_server.start()
        self.queue_size=10
        self.left_distance = 0
        self.current_error=-1
        self.average_error=0
        self.desired_distance_to_follow=0
        self.publisher = rospy.Publisher('/burger_commands/is_on_signal',Bool,queue_size=self.queue_size)
        self.laser = rospy.Subscriber('/scan',LaserScan,self.scan_callback)
        self.rate = rospy.Rate(200)

        self.counter = 0
        rospy.loginfo("server: Simple Action Server has been started")


    def on_goal(self, goal):
        rospy.loginfo("server: A goal has been received!")
        rospy.loginfo(goal)
        self.desired_distance_to_follow=goal.distance_to_follow

        success = False
        preempted = False
        is_on_signal = Bool()
        while not rospy.is_shutdown() and (not preempted):
                

                
            is_on_signal = True
            self.publisher.publish(is_on_signal)

            if self.action_server.is_preempt_requested():
                preempted = True
            else:
                rospy.loginfo(self.left_distance)
                feedback = StartFeedback()
                feedback.current_distance = self.left_distance
                feedback.current_error = self.current_error
                self.action_server.publish_feedback(feedback)
                self.rate.sleep()



        if  preempted:
            success = True
        self.send_result(success, preempted)


    def send_result(self, success, preempted):
        result = StartResult()
        result.average_error = self.average_error
        rospy.loginfo("server: send goal result to client")

        if preempted:
            rospy.loginfo("server: Preempted")
            self.action_server.set_preempted(result)
        elif success:
            rospy.loginfo("server: Success")
            self.action_server.set_succeeded(result)
        else:
            rospy.loginfo("server: failure")
            self.action_server.set_aborted(result)

    def scan_callback(self,data):
        laser_data = data.ranges
        self.left_distance = laser_data[-1]
        self.left_distance =laser_data[0]
        errors =[]
        if self.desired_distance_to_follow == 0:
            errors =[]
            self.current_error=-1
        else:
            errors.append(abs(laser_data[-1]-self.desired_distance_to_follow))
            self.current_error=errors[-1]
            self.average_error = sum(errors)/len(errors)

        
    def start(self):        
        rospy.spin()
