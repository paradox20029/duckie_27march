#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 
from math import atan2, sqrt, pi

class TurtlesimStraightsAndTurns:
    def init(self):

        # Initialize class variables
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.goal_x = 0
        self.goal_y = 0
        self.goal_theta = 0

        self.dist_goal_active = False
        self.angle_goal_active = False

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        rospy.Subscriber("/goal_pose", Pose, self.goal_pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def goal_pose_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_theta = msg.theta

        # Set goal active flags
        self.dist_goal_active = True
        self.angle_goal_active = True if abs(self.goal_theta - self.current_theta) > 0.01 else False

    def timer_callback(self, msg):
        if self.dist_goal_active:
            # Calculate distance to goal
            distance_to_goal = sqrt((self.goal_x - self.current_x) * 2 + (self.goal_y - self.current_y) * 2)

            # Move towards the goal only if it's within the arena
            if 0 <= self.goal_x <= 11 and 0 <= self.goal_y <= 11:
                # Calculate angle to goal
                angle_to_goal = atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

                # Calculate angle difference
                angle_difference = angle_to_goal - self.current_theta

                # Normalize angle difference to [-pi, pi]
                if angle_difference > pi:
                    angle_difference -= 2 * pi
                elif angle_difference < -pi:
                    angle_difference += 2 * pi

                # Set linear velocity based on distance to goal
                linear_velocity = min(distance_to_goal, 1.0)

                # Set angular velocity based on angle difference
                angular_velocity = angle_difference

                # Publish velocity commands
                vel_msg = Twist()
                vel_msg.linear.x = linear_velocity
                vel_msg.angular.z = angular_velocity
                self.velocity_publisher.publish(vel_msg)
            else:
                rospy.loginfo("Goal position is outside the arena!")

            # Check if goal reached
            if distance_to_goal < 0.1:
                rospy.loginfo("Goal reached!")
                self.dist_goal_active = False

        if self.angle_goal_active:
            # Calculate angle difference
            angle_difference = self.goal_theta - self.current_theta

            # Normalize angle difference to [-pi, pi]
            if angle_difference > pi:
                angle_difference -= 2 * pi
            elif angle_difference < -pi:
                angle_difference += 2 * pi

            # Set angular velocity based on angle difference
            angular_velocity = angle_difference

            # Publish angular velocity command
            vel_msg = Twist()
            vel_msg.angular.z = angular_velocity
            self.velocity_publisher.publish(vel_msg)

            # Check if goal angle reached
            if abs(angle_difference) < 0.01:
                rospy.loginfo("Goal angle reached!")
                self.angle_goal_active = False

if __name__ == '__main__': 
    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass

# rostopic pub /goal_x std_msgs/Float64 "data: 5.0"
# rostopic pub /goal_y std_msgs/Float64 "data: 8.0"
# rostopic pub /goal_angle std_msgs/Float64 "data: 1.57"  # 90 degrees in radians