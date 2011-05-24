#!/usr/bin/env python

################################################################################
## This file is for closed loop control of a single puppeteer with a
## single suspended mass.  It reads in the results of an optimization,
## and it subscribes to an estimator topic.  Based on the system state
## published by the estimator, and the optimization parameters that
## were read in, it then requests a service call to a serial node
## which then sends commands to the robot
################################################################################

import re
import roslib; roslib.load_manifest('puppeteer_control')
import rospy
import struct
from math import cos, sin, tan, pi
import numpy as np
import sys
import os
from std_msgs.msg import String
import puppeteer_msgs.msg as pmsg
import puppeteer_msgs.srv as psrv

defintion = {'__builtings__' : __builtins__}


################################################################################
################################################################################
## Global Variable Definitions:
################################################################################
################################################################################

## Optimization Parameters:
xopt = [] ## Optimal state
uopt = [] ## Optimal inputs
Kt = [] ## Optimal feedback gain
dt = 0.0 ## Timestep for the textfiles 
tf = 0.0 ## Total simulation time
Length = 0 ## Number of entries in each vector

## Transformation parameters for system
R = []
trans = []
robot_trans = []

## Time variables:
t_current = 0.0
t_last = 0.0

## Publisher names:
actual_state_pub = rospy.Publisher('transformed_state', pmsg.State)
desired_state_pub = rospy.Publisher('desired_state', pmsg.State)

## Service names:
serial_client = rospy.ServiceProxy('speed_command', psrv.speed_command)
serial_srv = psrv.speed_commandRequest

## Number of data points we want to collect for figuring out
## coordinate transformations:
npts = 30


################################################################################
################################################################################
## Structure Definitions:
################################################################################
################################################################################



################################################################################
################################################################################
## Function Definitions:
################################################################################
################################################################################

def build_dictionary(filename):
    """
    This function reads a file that was generated my Mathematica to
    create a dictionary that contains all of the results of an
    optimization.
    """
    f = open(filename, 'rU')
    ## Now, let's read the system parameters:
    line = f.readline()
    match = re.search(r'[0-9].*',line)
    dt = float(line[match.start():])
    line = f.readline()
    tf = float(line[match.start():])
    line = f.readline()
    Length = float(line[match.start():])

    ## Now, we are ready to start reading in the rest of the values:
    ## First, the optimal trajectory
    f.readline()
    xtemp = []
    for lines in f:
        match = re.search(r'.*',lines)
        if match: lines = match.group()
        if not re.search(r'[0-9].*',lines): break
        nums = lines.split(',')
        xtemp = []
        for num in nums:
            match = re.search(r'[0-9].*',num)
            if match:
                xtemp.append(float(num))
        xopt.append(xtemp)

    ## Now, the optimal inputs:
    utemp = [];
    for lines in f:
        match = re.search(r'.*',lines)
        if match: lines = match.group()
        if not re.search(r'[0-9].*',lines): break
        nums = lines.split(',')
        utemp = []
        for num in nums:
            match = re.search(r'[0-9].*',num)
            if match:
                utemp.append(float(num))
        uopt.append(utemp)

    ## Now the time varying gain matrix
    Ktemp = [];
    for lines in f:
        match = re.search(r'.*',lines)
        if match: lines = match.group()
        if not re.search(r'[0-9].*',lines): break
        nums = lines.split(',')
        Ktemp = []
        for num in nums:
            match = re.search(r'[0-9].*',num)
            if match:
                Ktemp.append(float(num))
        Kt.append(Ktemp)
    return [xopt, uopt, Kt]

def ros_setup():
    """
    This function defines all subscribers and publishers, and then
    starts the ros.spin infinite loop
    """
    ## We first define this to be a subscriber to the estimator node:
    rospy.loginfo("Starting Closed-Loop Control Node...")
    rospy.init_node('closed_loop_controller', anonymous=True)
    rospy.Subscriber("system_state", pmsg.State, estimator_callback)
    
    rospy.spin()


def estimator_callback(data):
    """
    This is the function that gets called when the estimator node
    publishes a new system state.  It is responsible for transforming
    the published state to the correct coordinate system, looking up
    the desired system state at the current time, and then publishing
    both of these.  It then looks at the error between the desired
    state and the actual state, and sends commands based on this error
    and the optimal feedback gain to the serial node
    """
    ## Name state published by estimator:

    ## Get current operating state:
    if rospy.has_param("operating_condition"):
        running_flag = rospy.get_param("operating_condition")

    if (running_flag == 0): ## we are then in idle mode:
        start_flag = True
        stop_flag = False
        return
    elif (running_flag == 1):
        ## Then it is time to start driving the robot
        if (start_flag == True):
            ## If we have just started, then let's send the command to
            ## unlock the robot:
            serial_srv.robot_index = 2
            serial_srv.type = 'm'
            serial_srv.Vleft = 0.0
            serial_srv.Vright = 0.0
            serial_srv.Vtop = 0.0
            serial_srv.div = 0
            start_flag == False
            call_count = 0
            avg_mass_pos = np.array([0.0, 0.0, 0.0])            
        else:
            ## For the first few times through this loop, let's just
            ## collect the data so that we can define the necessary
            ## transformation parameters:
            if call_count <= npts:
                call_count+=1
                avg_mass_pos += np.array([data.xm, data.ym, 0.0])/call_count
            elif call_count == npts+1:
                ## Then the data has been collected, and we can set
                ## the global transformation parameters:
                R = np.array([[cos(pi), sin(pi), 0],
                              [-sin(pi), cos(pi) 0],
                              [0, 0, 1]])
                

    ## If we are now running, let's transform the published state to
    ## the correct coordinate system:

    ## Now, let's get the desired state by interpolating the data that
    ## was read in from Mathematica

    ## Now, let's publish both the actual state and the desired state:

    ## Let's now use the feedback gain and the state error to determine
    ## what the new controls should be:

    ## Call the service for the serial node to send out these controls:


def transorm_state():
    """
    This function takes in the system state published from the
    estimator node and transforms it to the correct coordinate system.
    """
    

def interpolate_optimal():
    """
    This function takes in a current time, and uses it along with the
    data read in from Mathematica and some interpolation to return the
    desired state, controls, and feedback gain.
    """

def calculate_controls():
    """
    This function takes in a current state, an optimal state, and a
    feedback gain to determine the controls that should be sent to the
    robot.  In the optimization framework, these controls are
    accelerations, but in the actual robot, we are controlling
    velocities.  So it then integrates these controls, and converts
    them from translational velocities to rotational velocities using
    the geometry of the robot.

    It then returns these values.
    """

def main():
    """
    This is the main loop.  It makes variable declarations, and starts
    ros.spin()
    """
    ## The first thing that we do is call the text reading function:
    DIR = os.popen("rospack find puppeteer_control").read()
    DIR = DIR[0:-1]+"/data/"
    build_dictionary(DIR+"OptimalData.txt")

    ## Now let's check for the keyboard_node parameter:
    if (rospy.has_param("operating_condition")):
        rospy.set_param("operating_condition", 0)
    else:
        rospy.logwarn("Cannot Find Parameter: operating_condition")
        sys.exit(1)

    ## Call a function that initializes the publisher and subscriber:
    try:
        ros_setup()
    except rospy.ROSInterruptException: pass


if __name__=='__main__':
    main()
