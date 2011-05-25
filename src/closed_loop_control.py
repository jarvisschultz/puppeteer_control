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
import ctypes, time
from ctypes import util

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
robot_height = 0.0

## Time variables:
t_current = 0.0
t_last = 0.0

## Publisher names:
actual_state_pub = rospy.Publisher('transformed_state', pmsg.State)
desired_state_pub = rospy.Publisher('desired_state', pmsg.State)
actual_state = pmsg.State
desired_state = pmsg.State

## Service names:
serial_client = rospy.ServiceProxy('speed_command', psrv.speed_command)
robot_index = 2
div = 3
Vleft = 0.0
Vright = 0.0
Vtop = 0.0
msgtype = 'm'

## Number of data points we want to collect for figuring out
## coordinate transformations:
npts = 30

## Robot geometry:
Dwheel = 3.0*0.0254
Dpulley = 0.75*.0254

## Input values:
u_last = 0.0
v_last = 0.0
u_current = 0.0
v_current = 0.0

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
    match = re.search(r'[0-9].*',line)
    tf = float(line[match.start():])
    line = f.readline()
    match = re.search(r'[0-9].*',line)
    Length = float(line[match.start():])
    line = f.readline()
    match = re.search(r'[0-9].*',line)
    robot_height = float(line[match.start():])

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
            rospy.loginfo("Sending start string")
            ## If we have just started, then let's send the command to
            ## unlock the robot:
            msgtype = 'm'
            Vleft = 0.0
            Vright = 0.0
            Vtop = 0.0
            div = 0
            start_flag == False
            call_count = 0
            avg_mass_pos = np.array([0.0, 0.0, 0.0])
            avg_robot_pos = 0.0
        else:
            rospy.loginfo("Collecting transformation data")
            ## For the first few times through this loop, let's just
            ## collect the data so that we can define the necessary
            ## transformation parameters:
            if call_count <= npts:
                call_count+=1
                avg_mass_pos += np.array([data.xm, data.ym, 0.0])/call_count
                avg_robot_pos += data.xc/call_count
            elif call_count == npts+1:
                ## Then the data has been collected, and we can set
                ## the global transformation parameters:
                R = np.array([[cos(pi), sin(pi), 0],
                              [-sin(pi), cos(pi), 0],
                              [0, 0, 1]])
                mass_start = np.array([xopt[0][0],xopt[0][1],0])
                trans = mass_start-np.dot(R,avg_mass_pos)
                robot_start = xopt[0][2]
                robot_trans = robot_start-avg_robot_pos
                call_count += 1
                time_flag = True
            else:
                ## Now we are ready to start running the robot
                rospy.loginfo("Beginning movement execution")
                ## Get necessary base time info:
                if time_flag == True:
                    time_flag = False
                    t = rospy.Time
                    tbase = t.now()
                    tbase = tbase.to_sec()
                    t_current = 0.0
                    ## Let's just send the feedforward controls:
                    msgtype = 'h'
                    Vleft = (xopt[0][6])/(Dwheel/2.0)
                    Vright = (xopt[0][6])/(Dwheel/2.0)
                    Vtop = (xopt[0][7])/(Dpulley/2.0)
                    div = 3
                else:
                    ## Now, each time we hit this else, let's get the
                    ## total time elapsed, and the dt value
                    t_last = t_current
                    t_current = t.now()
                    t_current = t_current.to_sec()-t_base
                    tstep = t_current-t_base
                    ## Now we can transform the state published by the
                    ## estimator into the correct coordinate system
                    transorm_state(data)
                    ## Now, let's get the desired state by
                    ## interpolating the data that was read in from
                    ## Mathematica
                    uk,xk,Kk = interpolate_optimal(t_current)
                    ## Now, let's publish both the actual state and
                    ## the desired state:
                    actual_state_pub.(actual_state)
                    desired_state_pub.(desired_state)
                    ## Let's now use the feedback gain and the state
                    ## error to determine what the new controls should
                    ## be:
                    calculate_controls(uk,xk,Kk)

    ## Everytime this loop is done we call the service:
    try:
        resp = serial_client(robot_index, msgtype, Vleft, Vright, Vtop, div)
    except rospy.ServiceException:
        rospy.logwarn("Failed to call service: speed_command")

    if resp == False:
        rospy.logdebug("Send Successful: speed_command")
    else:
        rospy.logdebug("Send Request Denied: speed_command")

    return
        
def transorm_state(data):
    """
    This function takes in the system state published from the
    estimator node and transforms it to the correct coordinate system.
    """
    ## First, let's use the global transformation information to
    ## transform the mass positions and velocities
    pub_pos = np.array([data.xm, data.ym, 0.0])
    new_pos = (dot(R,pub_pos)+trans).tolist()
    actual_state.xm = new_pos[0]
    actual_state.ym = new_pos[1]
    ## Now, we can do the same thing with the velocities:
    pub_vel = np.array([data.xm_dot, data.ym_dot, 0.0])
    new_vel = (dot(R,pub_vel)).tolist()
    actual_state.xm_dot = new_vel[0]
    actual_state.ym_dot = new_vel[1]

    ## Now, we need to transform the robot position to fill it in:
    actual_state.xc = data.xc+robot_trans
    new_robot = np.array([actual_state.xc, robot_height, 0.0])
    actual_state.xc_dot = data.xc_dot

    ## Now, we need to get the string length and velocity:
    string_len = np.linalg.norm(new_robot-new_pos)
    actual_state.r = string_len
    mass_rel_vel = np.array([data.xm_dot-data.xc_dot,data.ym_dot])
    vec = np.array([data.xm-data.xc,data.ym-robot_height])
    norm_vec = vec/np.linalg.norm(vec)
    actual_state.r_dot = np.dot(mass_rel_vel,norm_vec)

    ## Now set velocity values for integration purposes:
    v_last = np.array([actual_state.xc_dot, actual_state.r_dot])

    return

def interpolate_optimal(t_current):
    """
    This function takes in a current time, and uses it along with the
    data read in from Mathematica and some interpolation to return the
    desired state, controls, and feedback gain.
    """
    ## So given a time we need to find the entries in the optimal
    ## state, inputs and feedback gains between which we need to
    ## interpolate the values
    index, rem = divmod(t_current, dt)
    index = int(index)
    if index > Length:
        rospy.logwarn("Trying to interpolate time greater than final time")
        return

    ## Interpolate u vals:
    uk = np.array(uopt[index-1])
    ukp = np.array(uopt[index])
    uinterp = uk+rem*(ukp-uk)/dt

    ## Now state values
    xk = np.array(xopt[index-1])
    xkp = np.array(xopt[index])
    xinterp = xk+rem*(xkp-xk)/dt

    ## Now gain values:
    Kk = np.array(Kt[index-1])
    Kkp = np.array(Kt[index])
    Kinterp = Kk+rem*(Kkp-Kk)/dt

    ## Set desired state values:
    desired_state.xm = xinterp[0]
    desired_state.ym = xinterp[1]
    desired_state.xc = xinterp[2]
    desired_state.r = xinterp[3]
    desired_state.xm_dot = xinterp[4]
    desired_state.ym_dot = xinterp[5]
    desired_state.xc_dot = xinterp[6]
    desired_state.r_dot = xinterp[7]
    
    return uinterp, xinterp, Kinterp
    

def calculate_controls(uk,xk,Kk):
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
    ## First assemble the actual state into an array:
    d = actual_state
    xactual = np.array([d.xm,d.ym,d.xc,d.r,d.xm_dot,d.ym_dot,d.xc_dot,d.r_dot])
    ## Now transform the feedback gain into a matrix:
    Kmat = np.array([Kk[0:8],Kk[8:]])
    ## Now get the input values:
    u_last = u_current
    u_current = np.array(uk)+np.dot(Kmat,xactual-xk)
    ## Now, we need to integrate this to get the type of values we want 
    ## v_last = v_current
    v_current = np.array((v_last+(u_last+u_current)/2.0*tstep)).tolist()
    ## Transform linear velocities to angular velocities:
    msgtype = 'h'
    Vleft = v_current[0]/(Dwheel/2.0)
    Vright = v_current[0]/(Dwheel/2.0)
    Vtop = v_current[1]/(Dpulley/2.0)
    div = 3

    return

def main():
    """
    This is the main loop.  It makes variable declarations, and starts
    ros.spin()
    """
    ## dll_name = "libkbhit_dll.so"
    ## dllabspath = os.path.dirname(os.path.abspath(__file__))+os.path.sep+dll_name
    ## try:
    ##     kblib = ctypes.CDLL(dllabspath)
    ## except:
    ##     print "Not loaded"

    ## try:
    ##     kbhit = kblib.kbhit
    ## except:
    ##     print "Function not found"

    ## while 1:
    ##     x = kbhit()
    ##     if x == 1:
    ##         print "Button Press!"
    ##         val = sys.stdin.readline()
    ##         print val
        
    ## The first thing that we do is call the text reading function:
    DIR = os.popen("rospack find puppeteer_control").read()
    DIR = DIR[0:-1]+"/data/"
    build_dictionary(DIR+"OptimalData.txt")

    ## Now let's check for the keyboard_node parameter:
    if (rospy.has_param("operating_condition")):
        rospy.set_param("operating_condition", 0)
    else:
        rospy.logwarn("Cannot Find Parameter: operating_condition")
       
    ## Call a function that initializes the publisher and subscriber:
    try:
        ros_setup()
    except rospy.ROSInterruptException: pass


if __name__=='__main__':
    main()
