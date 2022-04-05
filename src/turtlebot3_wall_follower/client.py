import rospy
import actionlib

from turtlebot3_wall_follower.msg import StartAction
from turtlebot3_wall_follower.msg import StartActionGoal

class BurgerClient:
    def __init__(self):
        
        self.distance = rospy.get_param("distance")
        
        self.goal = rospy.wait_for_message('burger_commands/goal',StartActionGoal)
        self.action_client = actionlib.SimpleActionClient('burger_commands',StartAction)        
        self.action_client.wait_for_server()
        rospy.loginfo("client: Action server is up!")

    def send_goal_and_get_result(self):

        goal = StartActionGoal()
        if self.distance != "":
            goal.distance_to_follow = self.distance
            self.action_client.send_goal(goal, done_cb=self.done_callback,
                                        feedback_cb=self.feedback_callback)
            
        else:
            goal = self.goal
            self.action_client.send_goal(goal, done_cb=self.done_callback,
                                        feedback_cb=self.feedback_callback)
        rospy.loginfo("client: Goal has been sent.")



    def done_callback(self, status, result):
        rospy.loginfo("client: Status is : " + str(status))
        rospy.loginfo("client: Result is : " + str(result))

    def feedback_callback(self, feedback):
        rospy.loginfo(f"client: {feedback}")
    


    def start(self):
        self.send_goal_and_get_result()
        rospy.spin()