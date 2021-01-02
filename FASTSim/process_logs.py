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
labels = ['x (m)','y (m)','z (m)','xdot (m/s)','ydot (m/s)','zdot (m/s)']
NUMSTATES = 6

for x in range(0,NUMSTATES):
    plt.figure()
    plt.plot(time,data[:,x+1])
    plt.xlabel('Time (sec)')
    plt.ylabel(labels[x])
    plt.grid()
    pp.savefig()
pp.close()
