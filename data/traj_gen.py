#!/usr/bin/python -tt

import sys
import numpy as np
import textwrap
import os
import pylab as mp
from math import sin, cos, pi


tf = 8.0*pi
dt = 0.01
plot_flag = True


def main(fname):
    offset = 0.0
    global dt, tf, plot_flag
    f = open(fname,'w')

    tvec = np.linspace(0.0, tf, int(tf/dt))
    tref = tvec+offset

    ## Define reference trajectory:
    
    ## xref = (1/(2.0*pi))*tvec
    ## yref = 0.0*np.sin(tvec/2./2.)
    ## zref = 0.0*tvec
    
    ## for i in range(int(5.0/dt)):
    ##     tvec = np.append(tvec,tvec[-1]+dt)
    ##     tref = np.append(tref,tref[-1]+dt)
    ##     ## np.append(tvec,tvec[-1])
    ##     ## np.append(tref,tref[-1])
    ##     xref = np.append(xref,xref[-1])
    ##     yref = np.append(yref,yref[-1])
    ##     zref = np.append(zref,zref[-1])

    ## yref = 0.5*np.sin(tvec/2.)
    ## xref = 0.5*np.sin(tvec/2./2.)
    ## zref = 0.0*tvec
    
    ## xref = 0.5*np.cos(tvec/2.)-0.5
    ## yref = 0.5*np.sin(tvec/2.)
    ## zref = 0.0*tvec
    ## xref = 1.5-1.5*np.exp(-tvec)
    ## yref = 0.0*tvec
    ## zref = 0.0*tvec

    xref = 0.5*np.cos(tvec/2.+3.0*np.pi/2.)
    yref = 0.5*np.sin(tvec/2.+3.0*np.pi/2.)
    zref = 0.0*tvec

    vd, wd = get_curvature(tvec,xref,yref)
        
    ## Write out the length of data:
    f.write("num= "+str(len(tvec))+"\n");
    for i in range(len(tvec)):
        ## str1 = '{0: f},{1: f},{2: f},{3: f},{4: f},{5: f}\n'.format(
        ##     tvec[i], xref[i], yref[i], zref[i], zref[i], tref[i])
        ## str1 = '{0: f},{1: f},{2: f},{3: f},{4: f}\n'.format(
        ##     tvec[i], xref[i], yref[i], vd[i], wd[i])
        str1 = '{0: f},{1: f},{2: f},{3: f}\n'.format(
            tvec[i], xref[i], yref[i], 12.0)
        f.write(str1);
    f.close()

    ## Now let's copy that file to a new directory:
    ## cmd = "cp "+fname+" /home/jarvis/Dropbox/mathematica/TrajectoryGenerator/data/kalman_debugging/"
    ## os.system(cmd)
    
    if plot_flag:
        generate_plot(xref, yref, zref, tvec)

def generate_plot(xref, yref, zref, tref):
    mp.subplot(211)
    a = mp.plot(xref[1:-1], yref[1:-1], '-')#, lw=2)
    ## a = mp.plot(tref, xref, '*')
    mp.axis('equal')
    mp.ylabel('y (m)')
    mp.xlabel('x (m)')
    mp.grid(True)
    mp.subplot(212)
    mp.plot(tref, zref, 'r-')#, lw = 2)
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
    
    ## print "vd = ", vd[0:9]
    ## print "wd = ", wd[0:9]
    ## for i in range(50):
    ##     print "t = ",t[i],"th = ",th[i]

    return vd, wd
    
        
    

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
        

    ## print "Output filename is: ",fname
    ## print "dt = ",dt
    ## print "tf = ",tf
    main(fname)


    ## dxref = 1.5*np.exp(-tvec)

    ## t2 = np.linspace(0.0, tf, int(tf/0.03333))
    ## x2 = 1.5-1.5*np.exp(-t2)
    ## dx2 = np.array([0])
    ## for i in range(1,len(t2)):
    ##     dx2 = np.append(dx2, (x2[i]-x2[i-1])/(t2[i]-t2[i-1]))

    ## print len(dx2), len(t2)

    ## mp.subplot(211)
    ## a = mp.plot(tref, xref, '-', lw = 2)
    ## mp.ylabel('x (m)')
    ## mp.xlabel('t (m)')
    ## mp.grid(True)

    ## mp.subplot(212)
    ## mp.plot(tref, dxref, '-',t2, dx2,'*', lw = 2)
    ## mp.grid(True)
    ## mp.show()
