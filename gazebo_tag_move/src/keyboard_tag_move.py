#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   i : forward(+x)    
   , : backward(-x)
   j : left   
   l : right       
   t : up(+z)
   b : down(-z)

For rotation, hold down the shift key:
---------------------------
   I : roll    
   < : roll
   J : pitch   
   L : pitch       
   T : yaw
   B : yaw

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0,0,0),
        ',':(-1,0,0,0,0,0),
        'j':(0,1,0,0,0,0),
        'l':(0,-1,0,0,0,0),
        't':(0,0,1,0,0,0),
        'b':(0,0,-1,0,0,0),
        'I':(0,0,0,1,0,0),
        '<':(0,0,0,-1,0,0),
        'J':(0,0,0,0,1,0),
        'L':(0,0,0,0,-1,0),
        'T':(0,0,0,0,0,1),
        'B':(0,0,0,0,0,-1),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('keyboard_tag_move')

    speed = rospy.get_param("~speed", 0.05)
    turn = rospy.get_param("~turn", 0.1)
    x = 0
    y = 0
    z = 0
    x_angle = 0
    y_angle = 0
    z_angle = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                x_angle = moveBindings[key][3]
                y_angle = moveBindings[key][4]
                z_angle = moveBindings[key][5]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                x_angle = 0
                y_angle = 0
                z_angle = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed
            twist.linear.y = y*speed
            twist.linear.z = z*speed
            twist.angular.x = x_angle*turn
            twist.angular.y = y_angle*turn
            twist.angular.z = z_angle*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)