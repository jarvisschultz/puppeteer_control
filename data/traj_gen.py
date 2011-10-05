#!/usr/bin/python -tt

import sys
import numpy as np
import textwrap
import os
import pylab as mp
from math import sin, cos, pi


tf = 4.0*pi
dt = 0.01
plot_flag = True


def main(fname):
    global dt, tf, plot_flag
    f = open(fname,'w')

    tvec = np.linspace(0.0, tf, int(tf/dt))

    ## Define referenc trajectory:    
    xref = 0.5*np.cos(tvec/2.0)
    yref = 0.5*np.sin(tvec/2.0)
    get_curvature(tvec,xref,yref)
    zref = 0.5*np.sin(2.0*tvec)
    ## zref = 0.1*tvec
    ## xref = 0.25*np.sin(tvec)
    ## yref = 0.25*np.sin(tvec/2)
    ## xref = 1.0*(tvec)
    ## yref = 0.0*tvec
    
    ## Write out the length of data:
    f.write("num= "+str(len(tvec))+"\n");
    for i in range(len(tvec)):
        str1 = '{0: f},{1: f},{2: f},{3: f},{4: f}\n'.format(
            tvec[i], xref[i], yref[i], zref[i], zref[i])
        f.write(str1);
    f.close()

    if plot_flag:
        generate_plot(xref, yref, zref, tvec)

def generate_plot(xref, yref, zref, tref):
    mp.subplot(211)
    a = mp.plot(xref, yref, '-', lw=2)
    mp.axis('equal')
    mp.ylabel('y (m)')
    mp.xlabel('x (m)')
    mp.grid(True)
    mp.subplot(212)
    mp.plot(tref, zref, 'r-', lw = 2)
    mp.show()

def get_curvature(t, x, y):
    xd = []; yd = []; xdd = []; ydd = [];
    for i in range(len(t)-1):
        dt = t[i+1]-t[i]
        xd.append((x[i+1]-x[i])/dt)
        yd.append((y[i+1]-y[i])/dt)
    xd.append(xd[-1])
    yd.append(yd[-1])
    for i in range(len(t)-1):
        dt = t[i+1]-t[i]
        xdd.append((xd[i+1]-xd[i])/dt)
        ydd.append((yd[i+1]-yd[i])/dt)
    xdd.append(xdd[-1])
    ydd.append(ydd[-1])

    th = np.arctan2(yd,xd)

    vd = np.sqrt(np.power(xd,2.0)+np.power(yd,2.0))
    wd = (np.array(ydd)*np.array(xd)
          -np.array(xdd)*np.array(yd))/(np.power(xd,2.0)+np.power(yd,2.0))

    print "vd = ", vd[0:9]
    print "wd = ", wd[0:9]
    for i in range(50):
        print "t = ",t[i],"th = ",th[i]
        
                   
    
    
        
    

usage = """
%s [options]

Required Options:

Options:
\t-f sets filename 
\t-t sets timestep
\t-e sets final time
\t-p sets whether or not to plot
\t-h prints this message
""" % sys.argv[0]

def print_usage():
    print usage

if __name__ == "__main__" :
    abbreviations = {
        '-f' : '--filename',
        '-t' : '--time_step',
        '-e' : '--final_time',
        '-h' : '--help',
        '-p' : '--plot'}

    for i in range(len(sys.argv)):
        if sys.argv[i] in abbreviations:
            sys.argv[i] = abbreviations[sys.argv[i]]
            
    if '--filename' in sys.argv:
        fname = sys.argv[sys.argv.index('--filename')+1]
    else:
        fname = 'default.txt'

    if '--time_step' in sys.argv:
        dt = float(sys.argv[sys.argv.index('--time_step')+1])

    if '--final_time' in sys.argv:
        tf = float(sys.argv[sys.argv.index('--final_time')+1])

    if '--help' in sys.argv:
        print_usage()

    if '--plot' in sys.argv:
        plot_flag = True
        

    print "Output filename is: ",fname
    print "dt = ",dt
    print "tf = ",tf
    main(fname)


