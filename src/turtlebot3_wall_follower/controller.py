import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Burger:
    queue_size=10
    def __init__(self):
        self.laser_data = []
        self.publisher = rospy.Publisher('cmd_vel',Twist,queue_size=Burger.queue_size)
        self.subscriber = rospy.Subscriber('scan_filtered',LaserScan,self.scan_callback)
        

    def move_straight(self,rate=200):
        currentrate = rospy.Rate(rate)
        while not  rospy.is_shutdown():
            
            curent_twist = Twist()
            curent_twist.linear.x = 0.2
            self.publisher.publish(curent_twist)
            currentrate.sleep()

      


   
        



    def scan_callback(self,data):
        self.laser_data = data.ranges