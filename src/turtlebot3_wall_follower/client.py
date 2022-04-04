import rospy
import actionlib

from turtlebot3_wall_follower.msg import StartAction
from turtlebot3_wall_follower.msg import StartActionGoal

class BurgerClient:
    def __init__(self):
        # self.goal = None
        
        self.goal = rospy.wait_for_message('burger_commands/goal',StartActionGoal)
        self.action_client = actionlib.SimpleActionClient('burger_commands',StartAction)        
        self.action_client.wait_for_server()
        rospy.loginfo("client: Action server is up!")

    def send_goal_and_get_result(self):

        # while not rospy.is_shutdown():
        #     if self.goal.goal.distance_to_follow == None:
        #         rospy.loginfo("client: please enter a distance.")
        #     else:
        #         self.action_client.send_goal(self.goal, done_cb=self.done_callback,
        #             feedback_cb=self.feedback_callback)
        #         rospy.loginfo("client: Goal has been sent.")
        self.action_client.send_goal(self.goal, done_cb=self.done_callback,
                                        feedback_cb=self.feedback_callback)
        rospy.loginfo("client: Goal has been sent.")



    def done_callback(self, status, result):
        rospy.loginfo("client: Status is : " + str(status))
        rospy.loginfo("client: Result is : " + str(result))

    def feedback_callback(self, feedback):
        rospy.loginfo(f"client: {feedback}")
    
    # def goal_callback(self,data):
    #     self.goal=data.goal

    def start(self):
        self.send_goal_and_get_result()
        rospy.spin()