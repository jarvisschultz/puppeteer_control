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
import math as m
import numpy as np
import sys
from std_msgs.msg import String

defintion = {'__builtings__' : __builtins__}


################################################################################
################################################################################
## Global Variable Definitions:
################################################################################
################################################################################
xopt = []
uopt = []
Kt = []
dt = 0.0
tf = 0.0
Length = 0


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
    ## We first define this to be a subscriber to the estimator node:

    ## Now we define the publishers: We will publish the transformed
    ## robot state from the estimator node, and the optimal state from
    ## the reading function


def estimator_callback():

    ## If we are not in running mode, then we don't do anything:

    ## If we are in run mode, then for the first few times through
    ## this loop, let's just collect the data so that we define the
    ## necessary transformation parameters:

    ## If we are now running, let's transform the published state to
    ## the correct coordinate system:

    ## Now, let's get the desired state by interpolating the data that
    ## was read in from Mathematica

    ## Now, let's publish both the actual state and the desired state:

    ## Let's now use the feedback gain and the state error to determine
    ## what the new controls should be:

    ## Call the service for the serial node to send out these controls:


def transorm_state():

def interpolate_optimal():

def calculate_controls():

def main():
    """
    This is the main loop.  It makes variable declarations, and starts
    ros.spin()
    """
    ## The first thing that we do is call the text reading function:
    build_dictionary("../data/OptimalData.txt")

    ## Call a function that initializes the publisher and subscriber:
    try:
        ros_setup()
    except rospy.ROSInterruptException: pass


if __name__=='__main__':
    main()
