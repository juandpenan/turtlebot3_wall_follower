import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from turtlebot3_wall_follower.msg import StartActionGoal
from std_msgs.msg import Bool


class Burger:
    
    def __init__(self):
        full_param_name = rospy.search_param('autostart')
        self.auto_start= rospy.get_param(full_param_name)              
        # self.auto_start = rospy.get_param("autostart")
        self.crash_distance = 0.25
        self.queue_size=10
        #middle_distance = lidar info at 315º see: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/
        self.middle_distance = 0
        self.front_distance = 0
        self.steering_value = 0
        self.velocity_value = 0 
        self.current_error = 0
        self.average_error = 0
        self.left_distance = 0               
        self.goal_distance = 0.6
        #the horizontal component of the middle disntance        
        self.goal_distance_pid = (self.goal_distance / math.cos(math.pi/4))
        self.is_on_signal_value = False
        self.current_rate = rospy.Rate(200)
        self.current_twist = Twist()

        self.goal = rospy.Subscriber('/burger_commands/goal',StartActionGoal,self.goal_callback)
        self.laser = rospy.Subscriber('/scan_filtered',LaserScan,self.scan_callback)
        self.steering = rospy.Subscriber('/steering/control_effort',Float64,self.steering_callback)
        self.is_on_signal = rospy.Subscriber('/burger_commands/is_on_signal',Bool,self.on_signal_callback)
        self.velocity = rospy.Subscriber('/velocity/control_effort',Float64,self.velocity_callback)
         
        self.publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=self.queue_size)        
        self.goal_distance_setpoint = rospy.Publisher('/steering/setpoint',Float64,queue_size=self.queue_size)
        self.crash_distance_setpoint = rospy.Publisher('/velocity/setpoint',Float64,queue_size=self.queue_size)        
        self.current_left_distance = rospy.Publisher('/steering/state',Float64,queue_size=self.queue_size)
        self.current_front_distance = rospy.Publisher('/velocity/state',Float64,queue_size=self.queue_size)

        
    def find_a_wall(self):
        #temporary distance to enter the loop
        distance = 40

        while not  rospy.is_shutdown() and  (distance>self.crash_distance):
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

            self.goal_distance_setpoint.publish(self.goal_distance_pid)
            self.crash_distance_setpoint.publish(self.crash_distance)
            
            self.current_twist.linear.x= self.velocity_value            
            self.current_twist.angular.z = self.steering_value
            self.publisher.publish(self.current_twist)
            
            self.current_left_distance.publish(self.middle_distance)
            self.current_front_distance.publish(self.front_distance)
            
            self.current_rate.sleep()

    
    def follow_wall(self):
        
        while not self.is_on_signal_value and not self.auto_start :
            rospy.logwarn("waiting for a goal")
            rospy.sleep(4.)

        if self.is_on_signal_value or self.auto_start:            
            self.find_a_wall()
            self.align_with_wall()
            self.keep_distance()


   
        

    def on_signal_callback(self,data):
        self.is_on_signal_value = data.data       
    def steering_callback(self,data):
        self.steering_value = data.data
    def velocity_callback(self,data):
        self.velocity_value = data.data
    def goal_callback(self,data):
        safe_distance = 0.6
        max_distance = 1.2
        min_distance = 0.35
        if data.goal.distance_to_wall > max_distance or data.goal.distance_to_wall<= min_distance :
            self.goal_distance=safe_distance
            rospy.logwarn("invalid distance value, distance was set to %f"%(safe_distance))
        else:
            self.goal_distance=data.goal.distance_to_wall

        self.goal_distance_pid = (self.goal_distance / math.cos(math.pi/4))
    def scan_callback(self,data):
        laser_data = data.ranges        
        self.middle_distance = laser_data[int(len(laser_data)/2)]
        self.front_distance = laser_data[0]