#!/usr/bin/env python

import threading
import pygame
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
         w    
    a    s    d

j/k : set linear max speeds 1.5/1.0
"""


linearBindings = {
        'w':1,
        's':-1,
    }

angularBindings = {
        'a':1,
        'd':-1,
    }

speedBindings={
        'j':1.5,
        'k':1.0,
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def back0(val, step):
    if val > 0:
        val -= step
        if val < 0:
            val = 0

    elif val < 0:
        val += step
        if val > 0:
            val = 0

    return val


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    pygame.init()

    speed = rospy.get_param("~speed", 1.0)
    turn = rospy.get_param("~turn", 0.5)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))

        while(1):
            x = pub_thread.x
            y = pub_thread.y
            z = pub_thread.z
            th = pub_thread.th

            keys = pygame.key.get_pressed()

            if keys[pygame.K_w]:
                x += 0.05
                if x > speed:
                    x = speed
            elif keys[pygame.K_s]:
                x -= 0.05
                if x < -speed:
                    x = -speed
            else:
                x = back0(x, 0.1)

            if keys[pygame.K_a]:
                th += 0.05
                if th > turn:
                    th = turn
            elif keys[pygame.K_d]:
                th -= 0.05
                if th < -turn:
                    th = -turn
            else:
                th = back0(th, 0.1)

            if keys[pygame.K_j]:
                speed = 1.0
 
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        pygame.quit()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        