import rospy 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtlesimPositionControl:
    def __init__(self):
        # Initialize class variables
        self.current_pose = Pose()
        self.goal_pose = Pose()
        self.goal_received = False

        # Initialize the node
        rospy.init_node('turtlesim_position_control_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Start the main loop
        self.main_loop()

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.goal_received:
            self.update_turtle_motion()

    def set_goal_pose_from_input(self):
        x = float(input("Enter the goal position (x): "))
        y = float(input("Enter the goal position (y): "))
        self.set_goal_pose(x, y)

    def set_goal_pose(self, x, y):
        self.goal_pose.x = x
        self.goal_pose.y = y
        self.goal_received = True
        self.update_turtle_motion()

    def update_turtle_motion(self):
        # Calculate the distance and angle to the goal
        distance_to_goal = math.sqrt((self.goal_pose.x - self.current_pose.x)**2 + (self.goal_pose.y - self.current_pose.y)**2)
        angle_to_goal = math.atan2(self.goal_pose.y - self.current_pose.y, self.goal_pose.x - self.current_pose.x)

        # Calculate the difference between current angle and goal angle
        angle_diff = angle_to_goal - self.current_pose.theta

        # Ensure the angle is within the range [-pi, pi]
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        if angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Create a Twist message to send velocity commands
        twist_msg = Twist()

        # Adjust angular velocity to rotate in-place towards the goal
        twist_msg.angular.z = angle_diff

        # If the angle difference is small enough, move forward
        if abs(angle_diff) < 0.1:
            twist_msg.linear.x = 1.0

        # Publish the velocity command
        self.velocity_publisher.publish(twist_msg)

        # Check if the goal is reached
        if distance_to_goal < 0.1:
            rospy.loginfo("Goal reached!")
            self.goal_received = False

    def main_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if not self.goal_received:
                self.set_goal_pose_from_input()  # Prompt user for goal input
            rate.sleep()

if __name__ == '__main__':
    try:
        position_controller = TurtlesimPositionControl()
        rospy.spin() # Keep the script running
    except rospy.ROSInterruptException:
        pass
