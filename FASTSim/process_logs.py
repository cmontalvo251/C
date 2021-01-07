#!/usr/bin/python

import matplotlib.pyplot as plt
import sys
import os
import numpy as np
try:
    from pdf import *
except:
    print('You need pdf from Python.git')
    sys.exit()

#Clean Logs
os.system('./clean_logs')

#Compile software
os.system('make')

#Run Software
os.system('./FASTSim.exe')

#Read Log File
data = np.loadtxt('logs/0.txt')
print(np.shape(data))

##Create PDF Handle
pp = PDF(0,plt)

#Plot everything
time = data[:,0]

#For Point Mass in Space
#labels = ['x (m)','y (m)','z (m)','xdot (m/s)','ydot (m/s)','zdot (m/s)']
#NUMSTATES = 6
#For sixdof model
labels = ['x (m)','y (m)', 'z (m)','q0','q1','q2','q3','u (m/s)','v (m/s)','w (m/s)','p (rad/s)','q (rad/s)','r (rad/s)']
NUMSTATES = 13

for x in range(0,NUMSTATES):
    plt.figure()
    plt.plot(time,data[:,x+1])
    plt.xlabel('Time (sec)')
    plt.ylabel(labels[x])
    plt.grid()
    pp.savefig()


#For sixdof models
import sixdof as dof
q0123 = data[:,4:8]
ptp = dof.quat2euler(np.transpose(q0123))
labels = ['Roll (deg)','Pitch (deg)','Yaw (deg)']
for x in range(0,3):
	plt.figure()
	plt.plot(time,ptp[x]*180.0/np.pi)
	plt.xlabel('Time (sec)')
	plt.ylabel(labels[x])
	plt.grid()
	pp.savefig()

pp.close()
