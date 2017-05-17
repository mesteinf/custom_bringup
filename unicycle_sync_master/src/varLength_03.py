#!/usr/bin/env python

import rospy
import os.path
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


class varLength_03():

    # Declare Functions
    # Callback function for the Exosystem 04
    def Exo_callback(self, data):
        self.w1_04 = data.x
        self.w2_04 = data.y
        self.w3_04 = data.z

    # Callback function for Position 04 - also includes control signals
    def Position_callback(self, msg):
        self.x1_04 = msg.linear.x
        self.x2_04 = msg.linear.y
        self.x3_04 = msg.linear.z

        self.v_04 = msg.angular.x
        self.angv_04 = msg.angular.y
        self.time_04 = msg.angular.z

    def __init__(self):
        # initiliaze
        rospy.init_node('varLength_03', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        # Declare variables
        loopRate = 100
        L = 0.2
        a = 1
        b = 1
        a0 = 0
        b0 = 0
        pathLength = 2*math.pi*a
        gain = 5
        S = [[0 for x in range(3)] for y in range(3)]
        S[0][0] = 0
        S[0][1] = 1
        S[0][2] = 0
        S[1][0] = 0
        S[1][1] = 0
        S[1][2] = 0
        S[2][0] = 0
        S[2][1] = 0
        S[2][2] = 0
        Q = [1, 0, 0]
        self.w1 = 0
        self.w2 = 1
        self.w3 = 0
        self.x1 = 1.3
        self.x2 = -0.2
        self.x3 = 1.5508
        self.t = 0
        self.dt = 1/loopRate
        self.w1_04 = -1
        self.w2_04 = -1
        self.w3_04 = -1

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi
        #      pyhpto /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.Exo = rospy.Publisher('/Exo_03', Point, queue_size=10)

	# Create Subscribers
        rospy.Subscriber('/Exo_04', Point, self.Exo_callback, queue_size=1)
        rospy.Subscriber('/position_04', Twist, self.Position_callback, queue_size=1)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(loopRate)

        # open a file to store the data to
        raw_data = open('/home/turtlebot3/catkin_ws/src/unicycle_sync_master/bin/test_may_16.txt', "w")
        raw_data.write('Agent03, X-pos03, Y-pos03, Heading03, Velocity03, Angular Velocity03, Exostate_103, Exostate_203, Exostate_303, Discrete_Time03, Agent04, X-pos04, Y-pos04, Heading04, Velocity04, Angular Velocity04, Exostate_104, Exostate_204, Exostate_304, Discrete_Time04\n')

        # Twist is a datatype for velocity
        move_cmd = Twist()
        Exo_03 = Point()

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():

                # wait for tthe loop rate
                r.sleep()

                if self.w2_04 >= 0:
                        # Output Transformation
                        y = [x1 + L*math.cos(x3), x2 + L*math.sin(x3)]

                        # Co-ordinate Transformation
                        # s(y) = (y1-a0)^2/a^2 + (y2-b0)^2/b^2 - 1

                        # tangential states
                        xi = math.pow(((y[0] - a0)/a), 2) + math.pow(((y[1] - b0)/b), 2) - 1
                        dxi = [2*(y[0] - a0)/math.pow(a, 2), 2*(y[1] - b0)/math.pow(b, 2)]

                        # transversal states
                        # For the case of a circle only integral(f_anon) = R*lambdaStar
                        # arc length
                        eta_Cham_03 = math.atan2((y[1] - b0), (y[0] - a0))   # angle [-pi, pi)
                        if (eta_Cham_03 < 0):                                # angle [0, 2*pi)
                                eta_Cham_03 = 2*math.pi + eta_Cham_03
                        sigmaPrime_norm = math.sqrt(math.pow(b*math.cos(eta_Cham_03), 2) + math.pow(-a*math.sin(eta_Cham_03), 2))

                        deta = [-(a*math.sin(eta_Cham_03))/sigmaPrime_norm, (b*math.cos(eta_Cham_03))/sigmaPrime_norm]
                        eta_Cham_03 = a*eta_Cham_03                          # arc-length [0, pathLength)

                        error = eta_Cham_03 - (pathLength/(2*math.pi))*math.fmod((Q[0]*w1 + Q[1]*w2 + Q[2]*w3), 2*math.pi)
                        # MATLAB angle(exp(1j*(2*math.pi/pathLength)*error_A))
                        angle = math.atan2(math.sin((2*math.pi/pathLength)*error), math.cos((2*math.pi/pathLength)*error))

                        external_input = [-gain*xi, -gain*angle + (pathLength/(2*math.pi))*(w1*(Q[0]*S[0][0] + Q[1]*S[1][0] + Q[2]*S[2][0]) + w2*(Q[0]*S[0][1] + Q[1]*S[1][1] + Q[2]*S[2][1]) + w3*(Q[0]*S[0][2] + Q[1]*S[1][2] + Q[2]*S[2][2]))]
                        # MATLAB [math.cos(x3), math.sin(x3); -math.sin(x3)/L, math.cos(x3)/L]*([dh1_A;dh2_A]\(external_input_A));
                        det = dxi[0]*deta[1] - dxi[1]*deta[0]

                        tmp1 = (1/det)*(external_input[0]*deta[1] + external_input[1]*-dxi[1])
                        tmp2 = (1/det)*(external_input[0]*-deta[0] + external_input[1]*dxi[0])

                        control_input = [tmp1*math.cos(x3) + tmp2*math.sin(x3), tmp1*-math.sin(x3)/L + tmp2*math.cos(x3)/L]

                        # Write current data o file
                        raw_data.write('Agent_03, %f , %f , %f, %f, %f, %f, %f, %f, %f, %f, 04, %f , %f , %f, %f, %f, %f, %f, %f, %f\n' % (x1, x2, x3, control_input[0], control_input[1], w1, w2, w3, t, x1_04, x2_04, x3_04, v_04, angv_04, w1_04, w2_04, w3_04, time_04))

                        # publish the velocity/Exosystem
                        move_cmd.linear.x = control_input[0]
                        move_cmd.angular.z = control_input[1]
                        Exo_03.x = w1
                        Exo_03.y = w2
                        Exo_03.z = w3
                        self.cmd_vel.publish(move_cmd)
                        self.Exo.publish(Exo_03)

                        # State Dynamics
                        x1 = x1 + math.cos(x3)*control_input[0]*dt
                        x2 = x2 + math.sin(x3)*control_input[0]*dt
                        x3 = x3 + control_input[1]*dt

                        # Exosysem Dynamics
                        w1 = w1 + ((S[0][0]*w1 + S[0][1]*w2 + S[0][2]*w3) + (w1_04 - w1))*dt
                        w2 = w2 + ((S[1][0]*w1 + S[1][1]*w2 + S[1][2]*w3) + (w2_04 - w2))*dt
                        w3 = w3 + ((S[2][0]*w1 + S[2][1]*w2 + S[2][2]*w3) + (w3_04 - w3))*dt

                        t += dt

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        varLength_03()
    except:
        rospy.loginfo("Syncronization Agent 03 node terminated.")


