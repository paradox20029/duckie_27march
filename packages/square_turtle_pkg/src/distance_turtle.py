#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 
import math

class DistanceReader:
    def __init__(self):
        
        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Initialize subscriber: input the topic name, message type and callback signature  
        rospy.Subscriber("/turtle1/pose", Pose,self.callback)

        # Initialize publisher: input the topic name, message type and msg queue size
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    # Whenever a message is received from the specified subscriber, this function will be called
    def callback(self,msg):
        rospy.loginfo("Turtle Position: %s %s", msg.x, msg.y)

        ########## YOUR CODE GOES HERE ##########
        # Calculate the distance the turtle has travelled and publish it
        pos_x=msg.x
        pos_y=msg.y 

        distance=math.sqrt((pos_x)**2+(pos_y)**2)
        rospy.loginfo("Total distance: %s", distance)
        self.distance_publisher.publish(distance)
        ###########################################

if __name__ == '__main__': 

    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass


# import rospy 
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float64
# from turtlesim.msg import Pose
# import time 
# import math

# class DistanceReader:
#     def _init_(self):
#         # Initialize the node
#         rospy.init_node('turtlesim_distance_node', anonymous=True)

#         # Initialize subscriber: input the topic name, message type and callback signature  
#         rospy.Subscriber("/turtle1/pose", Pose, self.callback)

#         # Initialize publisher: input the topic name, message type and msg queue size
#         self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

#         # Initialize velocity publisher
#         self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

#         # Printing to the terminal, ROS style
#         rospy.loginfo("Initialized node!")
        
#         # This blocking function call keeps python from exiting until node is stopped
#         rospy.spin()

#     def callback(self, msg):
#         # Create a Twist message to turn
#         turn = Twist()
#         turn.linear.y = 1.0

#         move_forward = Twist()
#         move_forward.linear.x = 1.0

#         move_backwards = Twist()
#         move_backwards.linear.x = -1.0  # Move backward at 1 m/s

#         # Get initial position
#         initx = msg.position.x
#         inity = msg.position.y

#         rospy.loginfo("Initial Turtle Position: %s %s", initx, inity)

#         # Move forward for 1 second
#         self.velocity_publisher.publish(move_forward)
#         time.sleep(1)

#         # Stop moving
#         self.velocity_publisher.publish(Twist())

#         # Turn for 1 second
#         self.velocity_publisher.publish(turn)
#         time.sleep(1)
#         self.velocity_publisher.publish(Twist())

#         # Move backwards for 1 second
#         self.velocity_publisher.publish(move_backwards)
#         time.sleep(1)
#         self.velocity_publisher.publish(Twist())

#         # Calculate the distance the turtle has traveled and publish it
#         finx = msg.position.x
#         finy = msg.position.y

#         diffx = (finx - initx) ** 2
#         diffy = (finy - inity) ** 2

#         distance = math.sqrt(diffx + diffy)

#         rospy.loginfo("Total distance: %s", distance)

# if __name__ == '__main__': 
#     try: 
#         distance_reader_class_instance = DistanceReader()
#     except rospy.ROSInterruptException: 
#         pass