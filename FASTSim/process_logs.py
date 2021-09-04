#!/usr/bin/python3

import matplotlib.pyplot as plt
import sys
import os
import numpy as np
sys.path.append('/home/mjcobar13/Documents/GIT/CMontalvo251/Python/pdf')
print(sys.path)
try:
    from pdf import *
except:
    print('You need pdf from Python.git This is on my Github just git clone that repo and put pdf.py in this root or add to pythonpath')
    sys.exit()

if len(sys.argv) > 1:
    print(sys.argv)
    SIMULATE = int(sys.argv[1])
else:
    SIMULATE = 1

if SIMULATE == 1:
    #Clean Logs
    os.system('./clean_logs')
    #Compile software
    os.system('make')
    #Run Software
    os.system('./FASTSim.exe')

#Read Log File
data = np.loadtxt('logs/0.txt',delimiter=',')
print(np.shape(data))

##Create PDF Handle
pp = PDF(0,plt)

#Plot everything
time = data[:,0]

#For Point Mass in Space
#labels = ['x (m)','y (m)','z (m)','xdot (m/s)','ydot (m/s)','zdot (m/s)']
#NUMSTATES = 6
#For sixdof model
state_vars = ['x (m)','y (m)', 'z (m)','q0','q1','q2','q3','u (m/s)','v (m/s)','w (m/s)','p (rad/s)','q (rad/s)','r (rad/s)']
sensor_vars = ['Sensor x (m)','Sensor y (m)', 'Sensor z (m)','Sensor Roll (deg)','Sensor Pitch (deg)','Sensor Yaw (deg)','Sensor u (m/s)','Sensor v (m/s)','Sensor w (m/s)','Sensor p (rad/s)','Sensor q (rad/s)','Sensor r (rad/s)']
rcin_vars = ['Throttle RX (us)','Aileron RX (us)','Elevator RX (us)','Rudder RX (us)','Aux1 RX','Aux2 RX','Aux3 RX','Aux4 RX']
ctl_vars = ['Throttle Command (us)','Aileron Command (us)','Elevator Command (us)','Rudder Command (us)','Aux1 Command (us)','Aux2 Command (us)','Aux3 Command (us)','Aux4 Command (us)']
force_moment_vars = ['X (N)','Y (N)','Z (N)','L (N-m)','M (N-m)','N (N-m)']

NUMVARS = len(state_vars) + len(sensor_vars) + len(rcin_vars) + len(ctl_vars) + len(force_moment_vars)

labels = state_vars + sensor_vars + rcin_vars + ctl_vars + force_moment_vars

###Print All Vars Raw
for x in range(0,NUMVARS):
    plt.figure()
    plt.plot(time,data[:,x+1])
    plt.xlabel('Time (sec)')
    plt.ylabel(labels[x])
    print(labels[x])
    plt.grid()
    pp.savefig()

pp.close()
sys.exit()

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
