#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pow, sqrt, atan, pi#, atan2
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion #, quaternion_from_euler
"""
   TurtleBot3(buger) MAX SPEED
-----------------------------------
MAX Linear  Speed: 0.22(meter /sec)
MAX Angular Speed: 2.82(radian/sec)
"""
MAX_LIN_X   = 0.22
MAX_ANG_Z   = 2.82

TARGET_ID   = 9

RIGHT_ANGLE = 1.57075
TOLERANCE   = 0.05

class TurtleBot3:

    def __init__(self):
        rospy.init_node('move_tb3_2marker', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
        self.rate = rospy.Rate(10)

        self.lin_x = MAX_LIN_X / 2
        self.ang_z = MAX_ANG_Z / 2

        self.twist     = Twist()
        self.goal_pose = Pose()

        self.wise  = 1
        self.Kp_y  = 3.0

        self.is_found_marker  = False
        self.is_get_goal_pose = False





        """        x
                   |
             ------+------
             |\    |    /|                     /|
       case1 |0\   |   /0| case2              / |         1 radian = 57.2958 degree
             |  \  |  /  |                   /30|         1 degree = 0.0174533 radian
       +x,+y |   \0|0/   | +x,-y            /   |
             |    \|/    |               2 /    | sqrt(3)
       y-----+-----+-----+-----           /     |
             |    /|\    |               /      |
       -x,+y |   /0|0\   | -x,-y        /       |
             |  /  |  \  |             /60    90|
       case4 |0/   |   \0| case3      ----------+
             |/    |    \|                 1
             ------+------

        d = dist  = sqrt(abs(x) + abs(y))

        0 = angle = math.atan(abs(y) / abs(x))

        case 1:  0 =  math.atan(abs(y) / abs(x))
        case 2: -0 = -math.atan(abs(y) / abs(x))
        case 3: -(180 * 0.0174533 - 0) = -(pi - 0) = -(pi - math.atan(abs(y) / abs(x)))
        case 4:   180 * 0.0174533 - 0  =   pi - 0  =   pi - math.atan(abs(y) / abs(x))
        """


    def get_marker(self, msg):

        n = len(msg.markers)

        if(n == 0):
            print("maker is not found...")
            # step1: find marker
            self.twist.angular.z = 0.2
            self.pub.publish(self.twist)

        else:
            if(self.is_get_goal_pose == False):
              # step2: check ID
              if( msg.markers[0].id == TARGET_ID ):
                  self.stop_move()
                  # step3: set initial pose
                  if  ( msg.markers[0].pose.pose.position.x <= -0.01 ):
                      self.twist.angular.z =  0.1
                      self.pub.publish(self.twist)
                  elif( msg.markers[0].pose.pose.position.x >=  0.01 ):
                      self.twist.angular.z = -0.1
                      self.pub.publish(self.twist)
                  else:
                      self.stop_move()

                      pos_x, pos_y, theta = self.get_ar_pose(msg.markers[0].pose.pose)

                      self.goal_pose.x = pos_x
                      self.goal_pose.y = pos_y

                      if  (theta >  5.):
                          self.goal_pose.theta = theta - 2 * pi
                      elif(theta < -5.):
                          self.goal_pose.theta = theta + 2 * pi
                      else:
                          self.goal_pose.theta = theta

                      self.is_get_goal_pose = True

            else:
                self.move2marker()

        """
                  y                        z
                  ^  x                     ^
          marker  | /                      | robot
        (on wall) |/                       |
                  +------> z      x <------+
                                          /
                                         /
                                        y

          orientation x,y,z,w --+
                                +--> 4   +-------------------------+
        input orientaion of marker ----->|                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <--------|                         |
                                 +-- 3   +-------------------------+
                 r,p,y angle <---+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <--
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   |
                                         +------------+------------+
        """

    def get_ar_pose(self, data):

        q = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

        theta = euler_from_quaternion(q)[1]

        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = data.position.z
        pos_y = data.position.y

        return pos_x, pos_y, theta


    """
    def get_dist(self, x, y):
        return sqrt(pow(abs(x), 2) + pow(abs(y), 2))


    def get_angle(self, x, y):

        if  (x >= 0 and y >= 0): # case 1: +0
            return  atan(abs(y) / abs(x))

        elif(x >= 0 and y <  0): # case 2: -0
            return -atan(abs(y) / abs(x))

        elif(x <  0 and y <  0): # case 3: -(pi-0)
            return -(pi - atan(abs(y) / abs(x)))

        elif(x <  0 and y >= 0): # case 4:  (pi-0)
            return   pi - atan(abs(y) / abs(x))
        """

    def move2marker(self):

        if( self.goal_pose.x < 0 ):
            dist_x   = -self.goal_pose.x - 0.1 # before 10cm to marker
        else:
            dist_x   =  self.goal_pose.x - 0.1 # before 10cm to marker

        if( self.goal_pose.y < 0 ):
            dist_y   = -self.goal_pose.y
        else:
            dist_y   =  self.goal_pose.y

        angle     = self.goal_pose.theta

        if(angle < 0):
            angle = -angle
            wise  = -1
        else:
            wise  =  1

        twist = Twist()

        ########################################################

        time2turn = (RIGHT_ANGLE - angle) / self.ang_z

        twist.angular.z = self.ang_z * wise
        time2end = rospy.Time.now() + rospy.Duration(time2turn)

        self.pub.publish(twist)
        rospy.sleep(0.001)

        while(rospy.Time.now() < time2end):   pass

        twist.angular.z = 0
        self.pub.publish(twist)

        #########################################################

        time2go_y = dist_y * self.Kp_y / self.lin_x

        twist.linear.x = self.lin_x
        time2end = rospy.Time.now() + rospy.Duration(time2go_y)

        self.pub.publish(twist)
        rospy.sleep(0.001)

        while(rospy.Time.now() < time2end):   pass

        twist.linear.x = 0
        self.pub.publish(twist)

        #########################################################

        wise = -wise

        #########################################################

        time2turn = RIGHT_ANGLE / self.ang_z

        twist.angular.z = self.ang_z * wise
        time2end = rospy.Time.now() + rospy.Duration(time2turn)

        self.pub.publish(twist)
        rospy.sleep(0.001)

        while(rospy.Time.now() < time2end):   pass

        twist.angular.z = 0
        self.pub.publish(twist)

        #########################################################

        time2go_x = dist_x / self.lin_x

        twist.linear.x = self.lin_x
        time2end = rospy.Time.now() + rospy.Duration(time2go_x)

        self.pub.publish(twist)
        rospy.sleep(0.001)

        while(rospy.Time.now() < time2end):   pass

        twist.linear.x = 0
        self.pub.publish(twist)

        #########################################################

        rospy.spin()


    def print_goal_pose(self):
        print("pos.x = %f,  pos.y = %f,  theta = %f" %(self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta))

        rospy.spin()


    def stop_move(self):
        self.twist.linear.x  = self.twist.linear.y  = self.twist.linear.z  = 0
        self.twist.angular.x = self.twist.angular.y = self.twist.angular.z = 0



if __name__ == '__main__':
    try:
        x = TurtleBot3()
        x.move2marker()
    except rospy.ROSInterruptException:   pass
