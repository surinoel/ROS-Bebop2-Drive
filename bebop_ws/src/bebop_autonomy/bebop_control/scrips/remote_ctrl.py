#!/usr/bin/env python
 
from __future__ import print_function
import rospy, os, time, sys, termios, select, atexit, tty, math
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool
from ftplib import FTP
import tempfile
 
msg = """
Control your Bebop2.0!
-----------------------------------------------------------
Left hand around:                    Right hand around:
        w                                    u                      
   a    s    d                         h     j     k

' ' : stop( hover )

w/s : going   up / down
a/d : rotate ccw / cw
i/k : go  foward / backward
j/l : go    left / righ
 
' ' : hovering
 1  : take off
 2  : landing
 3  : emergency
 
+/- : increase / decrease speed
 
Q to quit
"""
e = """
Communications Failed
"""
#         +---+-------------+---+-------------+---+-------------+---+-------------+---+------------+
#         |'w'| linear.z +  |'a'| angular.z + |'i'| linear.x +  |'j'| linear.y +  |' '|all param=0 |
#         +---+-------------+---+-------------+---+-------------+---+-------------+---+------------+
#         |'s'| linear.z -  |'d'| angular.z - |'k'| linear.x -  |'l'| linear.y +  |   |            |
#         +---+-------------+---+-------------+---+-------------+---+-------------+---+------------+
action = { 'w':( 0, 0, 1, 0),'a':( 0, 0, 0, 1),'i':( 1, 0, 0, 0),'j':( 0, 1, 0, 0),' ':( 0, 0, 0, 0),
           's':( 0, 0,-1, 0),'d':( 0, 0, 0,-1),'k':(-1, 0, 0, 0),'l':( 0,-1, 0, 0), }

# class for get 1 byte from standard input( keyboard )
class GetChar:
	def __init__(self):
		# Save the terminal settings
		self.fd = sys.stdin.fileno()
		self.new_term = termios.tcgetattr(self.fd)
		self.old_term = termios.tcgetattr(self.fd)

		# New terminal setting unbuffered
		self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
		termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

		# Support normal-terminal reset at exit
		atexit.register(self.set_normal_term)
	
	
	def set_normal_term(self):
		termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

	def getch(self):	    # get 1 byte from stdin
		""" Returns a keyboard character after getch() has been called """
		return sys.stdin.read(1)

	def chk_stdin(self):	# check keyboard input
		""" Returns True if keyboard character was hit, False otherwise. """
		dr, dw, de = select([sys.stdin], [], [], 0)
		return dr
 
def get_currunt_speed(lin_speed,ang_speed):
    return "currently:\tlinear speed: %s\tangular speed: %s " % (lin_speed, ang_speed)


if __name__=="__main__":
    # settings = termios.tcgetattr(sys.stdin)
 
    pub0 = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
    pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
    pub2 = rospy.Publisher('/bebop/land',    Empty, queue_size = 1)
    pub3 = rospy.Publisher('bebop/reset',   Empty, queue_size = 1)
    pub4 = rospy.Publisher('bebop/snapshot',Empty, queue_size = 1)
    pub5 = rospy.Publisher('bebop/record',  Bool,  queue_size = 1)
    
    empty_msg   = Empty()
    kb          = GetChar()    
    rospy.init_node('remote_ctrl')
 
    lin_speed   = rospy.get_param('~speed',  0.5)
    ang_speed   = rospy.get_param('~turn' ,  1.0)
    
    lin_offset  =  0.1;    ang_offset  =  math.radians(15)
    
    max_lin_spd =  1.5;    min_lin_spd = -1.5
    
    max_ang_spd =  math.radians(45)   #  45 degree 
    min_ang_spd = -math.radians(45)   # -45 degree    

    lin_x = 0;    lin_y = 0;    lin_z = 0;    ang_z = 0;    count = 0
 
    try:
        print(msg)
        print(get_currunt_speed(lin_speed,ang_speed))
        
        key = ''
        
        while(key != 'Q'):
        
            key = kb.getch()
            
            if key in action.keys():    # set linear x,y,z or angular z
                lin_x  = action[key][0]
                lin_y  = action[key][1]
                lin_z  = action[key][2] #
                ang_z  = action[key][3]
            
            elif key == '?':            # display help
                print(msg)
                
            elif key == '1':            # take off
                pub1.publish(empty_msg)
                
            elif key == '2':            # land
                pub2.publish(empty_msg)
                
            elif key == '3':            # emergency stop( just drop )
                pub3.publish(empty_msg)
            
            elif key == '=':            # linear & angular speed up
            
                if( lin_speed <= max_lin_spd - lin_offset ):
                    lin_speed = lin_speed + lin_offset
                else:
                    lin_speed = max_lin_spd
                    
                if( ang_speed <= max_ang_spd - ang_offset ):
                    ang_speed = ang_speed + ang_offset
                else:
                    ang_speed = max_ang_spd
            
            elif key == '-':            # linear & angular speed down
            
                if( lin_speed >= min_lin_spd + lin_offset ):
                    lin_speed = lin_speed - lin_offset
                else:
                    lin_speed = min_lin_spd
                    
                if( ang_speed >= min_ang_spd + ang_offset ):
                    ang_speed = ang_speed - ang_offset
                else:
                    ang_speed = min_ang_spd
                
            else:                       # just hover
                lin_x = 0;  lin_y = 0;  lin_z = 0;  ang_z = 0
            
            
            print(get_currunt_speed(lin_speed,ang_speed))
 
            twist = Twist()
            
            twist.linear.x  = lin_speed * lin_x
            twist.linear.y  = lin_speed * lin_y
            twist.linear.z  = lin_speed * lin_z
            
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = ang_speed * ang_z
            
            pub0.publish(twist)
        
        pub2.publish(empty_msg)         # land
        print("bebop have landed!")

    except KeyboardInterrupt:
        pub2.publish(empty_msg)         # land
        print("bebop have landed!")
        
