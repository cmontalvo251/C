#!/usr/bin/python3

import matplotlib.pyplot as plt
plt.rcParams.update({'figure.max_open_warning': 0})
import sys
import os
import numpy as np
#sys.path.append('/home/mjcobar13/Documents/GIT/CMontalvo251/Python/pdf')
#print(sys.path)

#MODELPATH='PortalCube/'
#MODELPATH='Quadcopter/'
#MODELPATH='Tank/'
MODELPATH='Airplane/'
#MODELPATH='X8/'
PLOTEVERYTHING = 0

try:
    from pdf import *
    import sixdof as dof
except:
    print('You need pdf and sixdof from Python.git This is on my Github just git clone that repo and put pdf.py and sixdof.py in this root or add to pythonpath')
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
    os.system('./FASTSim.exe ' + MODELPATH)

#Read Log File
data = np.loadtxt('logs/0.txt',delimiter=',')
r,c = np.shape(data)
print('Size of Data Set = ',r,c)

##Create PDF Handle
pp = PDF(0,plt)

######################Plot everything###########################3

#################3#Always log time#########
time = data[:,0]
########################################

##################GRAB COMPASS AND GPS HEADING######################
gps_heading = data[:,1]
compass = data[:,2]

#####The next 12 states are always the sensor states
#####################################################
sensor_states = data[:,3:15]
sensor_labels = ['Sensor x (m)','Sensor y (m)', 'Sensor z (m)','Sensor Roll (deg)','Sensor Pitch (deg)','Sensor Yaw (deg)','Sensor u (m/s)','Sensor v (m/s)','Sensor w (m/s)','Sensor p (deg/s)','Sensor q (deg/s)','Sensor r (deg/s)']

if PLOTEVERYTHING == 1:
    for x in range(0,12):
        plt.figure()
        plt.plot(time,sensor_states[:,x])
        plt.xlabel('Time (sec)')
        plt.ylabel(sensor_labels[x])
        print(sensor_labels[x])
        plt.grid()
        pp.savefig()
#######################################################

###############The Next N States are the RC Signals################
rcin_num_of_axis = int(data[:,15][0])
print('RCIN Num of Axis = ',rcin_num_of_axis)
rcin_labels = ['Throttle RX (us)','Aileron RX (us)','Elevator RX (us)','Rudder RX (us)','Aux1 RX','Aux2 RX','Aux3 RX','Aux4 RX']
rcin_states = data[:,16:(16+rcin_num_of_axis)]
for x in range(0,rcin_num_of_axis):
    plt.figure()
    plt.plot(time,rcin_states[:,x])
    plt.xlabel('Time (sec)')
    if x > len(rcin_labels):
        label = 'Aux' + ' ' + str(4+x-rcin_num_axis)
    else:
        label = rcin_labels[x]
    plt.ylabel(label)
    print(label)
    plt.grid()
    pp.savefig()
####################################################################

################THe next N states are the Control Signals###########
ctl_numsignals = int(data[:,16+rcin_num_of_axis][0])
print('Num Control Signals = ',ctl_numsignals)
ctl_states = data[:,(17+rcin_num_of_axis):(17+rcin_num_of_axis+ctl_numsignals)]
if PLOTEVERYTHING == 1:
    for x in range(0,ctl_numsignals):
        plt.figure()
        plt.plot(time,ctl_states[:,x])
        plt.xlabel('Time (sec)')
        label = 'PWM Output ' + str(x+1) + ' (us)'
        plt.ylabel(label)
        print(label)
        plt.grid()
        pp.savefig()
#####################################################################

##################The next columns only exist if you're running with
####################The RK4########################################
if 17+rcin_num_of_axis+ctl_numsignals < c:
    print('Integrator Detected')
    RK4 = 1
else:
    RK4 = 0
#######################################################################

########################The next 13 states are the actual states
if RK4 == 1:
    states_raw = data[:,(17+rcin_num_of_axis+ctl_numsignals):(30+rcin_num_of_axis+ctl_numsignals)]
    state_raw_labels = ['x (m)','y (m)', 'z (m)','q0','q1','q2','q3','u (m/s)','v (m/s)','w (m/s)','p (rad/s)','q (rad/s)','r (rad/s)']    
    if PLOTEVERYTHING == 1:
        ###Print All Vars Raw
        for x in range(0,13):
            plt.figure()
            plt.plot(time,states_raw[:,x])
            plt.xlabel('Time (sec)')
            plt.ylabel(state_raw_labels[x])
            print(state_raw_labels[x])
            plt.grid()
            pp.savefig()
    q0123 = states_raw[:,3:7]
    ptp = dof.quat2euler(np.transpose(q0123))
    ptp = np.transpose(ptp)*180.0/np.pi
    ptp_labels = ['Roll (deg)','Pitch (deg)','Yaw (deg)']
    ##PQR Must also be converted to deg/s
    pqr = states_raw[:,10:]*180.0/np.pi
    pqr_labels = ['p (deg/s)','q (deg/s)','r (deg/s)']
    if PLOTEVERYTHING == 1:
        for x in range(0,3):
            plt.figure()
            plt.plot(time,ptp[:,x])
            plt.xlabel('Time (sec)')
            plt.ylabel(ptp_labels[x])
            print(ptp_labels[x])
            plt.grid()
            pp.savefig()
    state_labels = state_raw_labels[0:3] + ptp_labels + state_raw_labels[7:10] + pqr_labels
    #print(np.shape(states_raw[:,0:3]))
    #print(np.shape(states_raw[:,7:]))
    #print(np.shape(ptp))
    states = np.concatenate((states_raw[:,0:3],ptp,states_raw[:,7:10],pqr),axis=1)
    #print(np.shape(states))
    if PLOTEVERYTHING == 1:
        ###Print All Vars Raw
        for x in range(0,12):
            plt.figure()
            plt.plot(time,states[:,x])
            plt.xlabel('Time (sec)')
            plt.ylabel(state_labels[x])
            print(state_labels[x])
            plt.grid()
            pp.savefig()

    #################The Next states only exist if there are actuators
    ##################First check to see if the actuator model is on
    simulation_flag_filename = MODELPATH+'Input_Files/Simulation_Flags.txt'
    file = open(simulation_flag_filename,'r')
    contents = file.read().split('\n')
    ACTUATORS = int(float(contents[10].split('!')[0]))
    file.close()
    #if actuators are on pull in NUMACTUATORS
    if ACTUATORS == 1:
        print("Using Actuators")
        actuatorfilename = MODELPATH+'Input_Files/Actuators.txt'
        file = open(actuatorfilename,'r')
        contents = file.read().split('\n')
        NUMACTUATORS = int(float(contents[1].split('!')[0]))
    else:
        print("No Actuators")
        NUMACTUATORS = 0
    print('Number of Actuators = ',NUMACTUATORS)
    if NUMACTUATORS > 0:
        ##The next N*2 states are the actual states of the actuators and the Error States
        actuator_labels = ['Throttle Actuator (us)','Aileron Actuator (us)','Elevator Actuator (us)','Rudder Actuator (us)','Aux1 Actuator (us)','Aux2 Actuator (us)','Aux3 Actuator (us)','Aux4 Actuator (us)']
        actuator_states = data[:,(30+rcin_num_of_axis+ctl_numsignals):(30+rcin_num_of_axis+ctl_numsignals+NUMACTUATORS)]
        actuator_error_states = data[:,(30+rcin_num_of_axis+ctl_numsignals+NUMACTUATORS):(30+rcin_num_of_axis+ctl_numsignals+2*NUMACTUATORS)]
        if PLOTEVERYTHING == 1:
            ###Print All Vars Raw
            for x in range(0,NUMACTUATORS):
                plt.figure()
                plt.plot(time,actuator_states[:,x],label='Truth')
                plt.plot(time,actuator_error_states[:,x],label='Error')
                plt.xlabel('Time (sec)')
                plt.ylabel(actuator_labels[x])
                print(actuator_labels[x])
                plt.grid()
                pp.savefig()
    #########################################################################

    #####################The Final 6 states are the Forces and moments on the Vehicle
    forces_moments = data[:,(30+rcin_num_of_axis+ctl_numsignals+2*NUMACTUATORS):(30+rcin_num_of_axis+ctl_numsignals+2*NUMACTUATORS+6)]
    force_moment_labels = ['X (N)','Y (N)','Z (N)','L (N-m)','M (N-m)','N (N-m)']
    ###Print All Vars Raw
    for x in range(0,6):
        plt.figure()
        plt.plot(time,forces_moments[:,x])
        plt.xlabel('Time (sec)')
        plt.ylabel(force_moment_labels[x])
        print(force_moment_labels[x])
        plt.grid()
        pp.savefig()
##########################################################################3

##### VARIABLE REGISTRY ####################
# time - DONE
# gps_heading , compass
# sensor_states and sensor_labels - 12x1 - DONE
# rcin_states and rcin_labels - rcin_num_of_axis x 1 - DONE
# ctl_states and ctl_labels - ctl_numsignals x 1 - DONE
# RK4 - 1 or 0 for on and off 
# The following states exist if RK4 == 1
# states_raw and states_raw_labels - 13x1 - ONLY PLOT IF PLOTEVERYTHING IS ON
# states and state_labels - 12x1 - DONE - DONE
# forces_moments and force_moment_labels - 6x1 - DONE
# NUMACTUATORS - 0 to N - number of actuators
# The following variables only exist if there are actuators
# actuator_states,actuator_error_states and actuator_labels - NUMACTUATORS x 1 - DONE

###Ok Let's first plot all 12 states (sensors and actual if RK4)
for x in range(0,12):
    plt.figure()
    plt.plot(time,sensor_states[:,x],label='Measurement')
    plt.xlabel('Time (sec')
    plt.grid()
    LEGEND=0
    if RK4 == 1:
        plt.plot(time,states[:,x],'--',label='Actual')
        plt.ylabel(state_labels[x])
        LEGEND=1
    else:
        plt.ylabel(sensor_labels[x])
    if x == 5:
        ##This is the yaw angle so we need to plot GPS heading and compass
        plt.plot(time,gps_heading,'-.',label='GPS')
        plt.plot(time,compass,'.',label='IMU+GPS')
        LEGEND = 1
    if LEGEND:
        plt.legend()
    pp.savefig()

##The RC Signals are plotted above 

##We now need to plot the control signals because if actuators are on they 
##will need to be plotted alongside the control signals
#Right now the code won't run unless the number of actuators is the same
##as the number of control signals
for x in range(0,ctl_numsignals): 
    plt.figure()
    plt.plot(time,ctl_states[:,x],label='Command')
    plt.xlabel('Time (sec)')
    label = 'PWM Output ' + str(x+1) + ' (us)'
    plt.ylabel(label)
    print(label)
    plt.grid()
    if NUMACTUATORS > 0:
        plt.plot(time,actuator_states[:,x],'--',label='Actuator')
        plt.plot(time,actuator_error_states[:,x],'-.',label='Actuator w/ Error')
        plt.legend()
    pp.savefig()

pp.close()
sys.exit()
