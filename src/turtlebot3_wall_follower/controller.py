import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from turtlebot3_wall_follower.msg import StartActionGoal
from std_msgs.msg import Bool

class Burger:
    
    def __init__(self):
        #distance in m that the robot will  follow
              
        
        self.desired_distance_to_avoid = 0.6
        self.queue_size=10
        self.laser_data = []
        self.middle_distance = 0
        self.front_distance = 0
        self.stering_value = 0
        self.velocity_value = 0 
        self.current_error = 0
        self.average_error = 0
        self.left_distance = 0 
        self.goal = rospy.Subscriber('/burger_commands/goal',StartActionGoal,self.goal_callback)
        self.desired_distance_to_follow = 0.6
        
        self.desired_distance_to_follow_pid = (self.desired_distance_to_follow / math.cos(math.pi/4))
        self.publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=self.queue_size)
        self.laser = rospy.Subscriber('/scan_filtered',LaserScan,self.scan_callback)
        self.distance_to_follow = rospy.Publisher('/steering/setpoint',Float64,queue_size=self.queue_size)
        self.distance_to_avoid = rospy.Publisher('/velocity/setpoint',Float64,queue_size=self.queue_size)
        self.steering = rospy.Subscriber('/steering/control_effort',Float64,self.stering_callback)
        self.is_on_signal = rospy.Subscriber('/burger_commands/is_on_signal',Bool,self.on_signal_callback)
        self.is_on_signal_value = False
        self.velocity = rospy.Subscriber('/velocity/control_effort',Float64,self.velocity_callback)
        self.current_left_distance = rospy.Publisher('/steering/state',Float64,queue_size=self.queue_size)
        self.current_front_distance = rospy.Publisher('/velocity/state',Float64,queue_size=self.queue_size)
        self.current_rate = rospy.Rate(200)
        self.current_twist = Twist()
        
    def find_a_wall(self):
        #temporary distance to enter the loop
        distance = 40

        while not  rospy.is_shutdown() and  (distance>self.desired_distance_to_avoid):
            distance = self.front_distance
            if distance == 0:
                distance = 40
            self.current_twist.linear.x = 0.2
            self.publisher.publish(self.current_twist)
            self.current_rate.sleep()

        self.current_twist.linear.x = 0
        self.publisher.publish(self.current_twist)



    def align_with_wall(self):
        self.current_twist.angular.z=-1.41
        self.publisher.publish(self.current_twist)
        rospy.sleep(1.5)
        self.current_twist.angular.z=0
        self.publisher.publish(self.current_twist)


    def keep_distance(self):

        
        
        
        while not  rospy.is_shutdown():

            self.distance_to_follow.publish(self.desired_distance_to_follow_pid)
            self.distance_to_avoid.publish(self.desired_distance_to_avoid)

            
            self.current_twist.linear.x= self.velocity_value            
            self.current_twist.angular.z = self.stering_value
            self.publisher.publish(self.current_twist)
            
            self.current_left_distance.publish(self.middle_distance)
            self.current_front_distance.publish(self.front_distance)
            
            self.current_rate.sleep()

    
    def follow_wall(self):

        while not self.is_on_signal_value:
            rospy.logwarn("waiting for a goal")

        if self.is_on_signal_value:
            
            self.find_a_wall()
            self.align_with_wall()
            self.keep_distance()
        else:
            rospy.logwarn("waiting for a goal")


   
        

    def on_signal_callback(self,data):
        self.is_on_signal_value = data
    def stering_callback(self,data):
        self.stering_value = data.data
    def velocity_callback(self,data):
        self.velocity_value = data.data
    def goal_callback(self,data):
        self.desired_distance_to_follow=data.goal.distance_to_follow
        self.desired_distance_to_follow_pid = (self.desired_distance_to_follow / math.cos(math.pi/4))

    def scan_callback(self,data):
        self.laser_data = data.ranges
        
        self.middle_distance = self.laser_data[int(len(self.laser_data)/2)]
        self.front_distance = self.laser_data[0]