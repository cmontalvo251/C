#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import sys
import os

##In order to import this toolbox into a python script you need to 
##do the following. Copy the following lines of code below
# import sys
# sys.path.append('/home/carlos/Dropbox/BlackBox/plotting')
# from gps import *

# or 
# In order to get python to search for all of your lovely blackbox 
# python routines. Add this to your .bashrc file

# for d in /home/carlos/Dropbox/BlackBox/*/; do
# 	PYTHONPATH+=:$d
# done
# export PYTHONPATH

# In order to get this to work in Thonny you need to navigate to
# /home/user/.thonny/BundledPython36/lib/python3.6/site-packages and place
# a symbolic link here

# In Enthough you need to make symbolic links here
# /home/carlos/.local/share/canopy/edm/envs/User/lib/python2.7/site-packages

NM2FT=6076.115485560000
FT2M=0.3048
GPSVAL = 60.0*NM2FT*FT2M
EarthRadius_ae = 6378140. ##meters mean eq radius
#23 hours 56 minutes 4 seconds
solar_day_sec = 23*3600.0 + 56*60.0 + 4.0
wEarth = 2*np.pi/solar_day_sec

def combineGPSArduinoTime(time_arduino,time_gps,pp,debugmode):

    time_gps_np = np.array(time_gps)

    #Get arduino and gps delta
    dt_gps = (time_gps_np[-1] - time_gps_np[0])*3600 #Assume gps is in hours and we convert to seconds
    dt_arduino = time_arduino[-1] - time_arduino[0]

    #Now we must scale the arduino time to match gps_time

    #Set pitot probe time to arduino time for now
    time_np = np.array(time_arduino)
    time_np = (time_np-time_np[0])*dt_gps/dt_arduino

    #Now recheck the arduino time_np
    #dt_arduino_new = time_np[-1]-time_np[0]
    #print dt_gps,dt_arduino,dt_arduino_new

    #With the pitot time shifted it's possible now to compute total gps+arduino time
    time_arduino_hr = time_np/3600
    time_arduino_hr_offset_gps = time_arduino_hr + time_gps_np[0]
    tot_time = []
    gps_offset = 0
    tot_time1 = time_gps_np[0]
    gps_old = time_gps_np[0]
    lastGPStime = 0

    for x in range(0,len(time_gps_np)):
        #Reset Arduino clock
        if time_gps_np[x] != gps_old:
            lastGPStime = time_arduino_hr[x]
            gps_old = time_gps_np[x]

        new_time = time_gps_np[x] + time_arduino_hr[x] - lastGPStime
 
        if len(tot_time) > 0:
            if new_time <= tot_time[-1]:
                lastGPStime = lastGPStime - tot_time[-1] + new_time
                new_time = time_gps_np[x] + time_arduino_hr[x] - lastGPStime
                #Double check for multiple times
                if new_time == tot_time[-1]:
                    new_time = new_time + 1e-8
            
        tot_time.append(new_time)

    tot_time_np = np.array(tot_time)

    tot_time_hr = tot_time_np
    tot_time_sec = tot_time_np*3600
    tot_time_sec_zero = tot_time_sec - tot_time_sec[0]

    if debugmode == 1:
        figureTime = plt.figure()
        plotTime = figureTime.add_subplot(1,1,1)
        plotTime.plot(time_gps_np,color='blue',label="GPS Time")
        plotTime.plot(time_arduino_hr_offset_gps,color='red',label="Scaled Arduino + GPS Offset Time")
        plotTime.plot(tot_time_np,color='green',label="GPS+Arduino Time")
        plotTime.set_xlabel('Row Number')
        plotTime.set_ylabel('Time (hrs)')
        plotTime.legend(loc=2)
        plotTime.grid()
        plotTime.get_yaxis().get_major_formatter().set_useOffset(False)
        pp.savefig()

    return tot_time_np


def NMEA_LAT_LON(IN):
    l = np.float(IN)/100.0;
    l_deg = np.floor(l);
    l_min = (l-l_deg)*100.0;
    out = l_deg + l_min/60.0;
    return out

def NMEA_TIME(time_raw,units):
    #try splitting by :
    times = time_raw.split(':')
    if len(times) == 1:
        #split didnt work which means there are no colons
        #print time_raw
        hour = np.float(time_raw[0:2])
        minute = np.float(time_raw[2:4])
        sec = np.float(time_raw[4:6])
        msec = 0
        #print hour,minute,sec
    else:
        hour = np.float(times[0])
        minute = np.float(times[1])
        sec = np.float(times[2])
        try:
            msec = np.float(times[3])
        except:
            msec = 0;
    time = 0
    if units == 'sec':
        time = msec/1000 + sec + minute*60 + hour*3600
    elif units == 'hrs':
        time = hour + minute/60 + sec/3600 + (msec/1000)/3600
    else:
        print('Invalid Unit in NMEA_TIME. Returning 0')
    #print time
    return time

###Functions
def computeGeocentricLATLON(x,y,z,t):
    xprime = np.sqrt(x**2 + y**2)
    zprime = z
    latitude = np.arctan2(zprime,xprime)*180/np.pi
    longitude = np.asarray(np.arctan2(y,x)*180.0/np.pi)-180.0/np.pi*wEarth*t
    #longitude[longitude<0]+=360.0
    #norm = np.sqrt(x**2 + y**2 + z**2)
    #phi = np.arccos(self.zsat / rho)
    #the = np.arctan2(self.ysat,self.xsat);
    #self.latitude = 90 - phi*(180 /np.pi);
    #self.longitude = the*(180/np.pi);
    return latitude,longitude

def IFOV(swath,deltas,az_rad):
    cosdtprime = np.cos(swath)*np.sin(deltas)+np.sin(swath)*np.cos(deltas)*np.cos(az_rad)
    dtprime = np.arccos(cosdtprime)
    dt = 90.0 - dtprime * 180.0/np.pi
    dt_rad = dt*np.pi/180.0
    cosdL = np.asarray((np.cos(swath)-np.sin(deltas)*np.sin(dt_rad))/(np.cos(deltas)*np.cos(dt_rad)))
    cosdL[abs(cosdL)>1.0] = 1.0
    dL = np.arccos(cosdL)*180/np.pi
    return dt,dL

def LATLON2Cartesian(lat,lon,alt):
    z = alt*np.sin(lat*np.pi/180.0)
    xyprime = alt*np.cos(lat*np.pi/180.0)
    x = xyprime*np.cos(lon*np.pi/180.0)
    y = xyprime*np.sin(lon*np.pi/180.0)
    return x,y,z

def convertXY2LATLON(xy,origin):
    global GPSVAL
    x = xy[0]
    y = xy[1]
    Ox = origin[0]
    Oy = origin[1]
    lat = x/GPSVAL + Ox
    lon = y/(GPSVAL*np.cos(Ox*np.pi/180)) + Oy
    return np.asarray([lat,lon])

def convertLATLON(lat_lon,origin):
    global GPSVAL
    lat = lat_lon[0]
    lon = lat_lon[1]
    Ox = origin[0]
    Oy = origin[1]
    x = (lat-Ox)*GPSVAL
    y = (lon-Oy)*GPSVAL*np.cos(Ox*np.pi/180)
    return np.asarray([x,y])

#Read data from file
def gps_data(filename):
    file = open(filename)

    lat_vec = []
    lon_vec = []
    time_vec = []
    alt_vec = []
    lenfile = 0

    for line in file:
        # lenfile+=1
        #Check to make sure a line was read
        if len(line) > 0:
            #Split the line into a list
            row = line.split(',')
            #check to make sure we've got data and we're reading GPRMC
            #also check for data
            if len(row) > 2 and row[0][5]=='C' and len(row[5])>0:
                time = NMEA_TIME(row[1],'hrs') 
                lat = NMEA_LAT_LON(row[3])
                lon = NMEA_LAT_LON(row[5])
                #print 'Time = ',time,' LAT = ',lat,' LONG = ',lon
                #Append to a vector
                time_vec.append(time)
                lat_vec.append(lat)
                lon_vec.append(lon)
            if len(row) > 2 and row[0][5]=='A' and len(row[5])>0:
                alt_vec.append(row[9])

    #Convert lists to numpy arrays. Make sure the list contains numpy floats
    lat_vec_np = np.array(lat_vec)
    lon_vec_np = np.array(lon_vec)
    time_vec_np = np.array(time_vec)
    alt_vec_np = np.array(alt_vec)
    #Convert to Cartesion Coordinates
    if len(lat_vec_np) > 1:
        origin = [lat_vec_np[0],lon_vec_np[0]]
    else:
        origin = [0,0]
    lat_lon = [lat_vec_np,lon_vec_np]
    xy = convertLATLON(lat_lon,origin)
    x_vec_np = xy[0]
    y_vec_np = xy[1]

    return [lat_vec_np,lon_vec_np,time_vec_np,x_vec_np,y_vec_np,alt_vec_np]

def HHMM_Format(time_vec,del_min):
    ##Assume this time_vec is in format HH.(HH/60)
    time_label = []
    last_time = time_vec[0]-del_min
    ctr = -1
    for time in time_vec:
        hour = int(np.floor(time))
        minute = int(np.round((time-hour)*60))
        #print time,hour,minute
        str_minute = str(minute)
        if (minute < 10):
            str_minute = '0' + str_minute
        ##Check and make sure enough time has passed
        str_time = str(hour)+':'+str_minute
        if (time-last_time)*60.0 >= del_min-1e-2:
            last_time = time
            #print str_time,ctr
            ctr+=1
            time_label.append(str_time)

    #print time_label
    xticks = np.linspace(time_vec[0],time_vec[-1],ctr+1)

    return time_label,xticks

def create_gps_plots(data,del_minute,pp,openfile=True):
    lat_vec_np = data[0]
    lon_vec_np = data[1]
    time_vec_np = data[2]
    x_vec_np = data[3]
    y_vec_np = data[4]
    alt_vec_np = data[5]

    #Assume this time_vec_np is in HH.(HH/60) but Sytske 
    #wants the data in HH:MM format so let's get a routine 
    #that does just that

    time_vec_HHMM,xticks = HHMM_Format(time_vec_np,del_minute)
    #linspace - start,end,number of points

    ###SAVE FIGS
    #pp = PdfPages('plots.pdf')

    #if len(lat_vec_np) == 0:
    #    return pp

    #Lon vs time
    figure1 = plt.figure()
    plot1 = figure1.add_subplot(1,1,1)
    plot1.plot(time_vec_np,lon_vec_np)
    plot1.set_xlabel('Time (HH:MM)')
    plot1.set_ylabel('Longitude (W)')
    plot1.grid()
    plot1.get_yaxis().get_major_formatter().set_useOffset(False)
    plot1.set_xticks(xticks) 
    plot1.set_xticklabels(time_vec_HHMM,rotation=0,ha='right',fontsize=12)
    axes = plt.gca()
    axes.set_xlim([time_vec_np[0],time_vec_np[-1]])    
    plt.gcf().subplots_adjust(left=0.15)
    pp.savefig()

    #Lat vs time
    figure2 = plt.figure()
    plot2 = figure2.add_subplot(1,1,1)
    plot2.plot(time_vec_np,lat_vec_np)
    plot2.set_xlabel('Time (HH:MM)')
    plot2.set_ylabel('Latitude (N)')
    plot2.grid()
    plot2.set_xticks(xticks) #linspace - start,end,number of points
    plot2.set_xticklabels(time_vec_HHMM,rotation=0,ha='right',fontsize=12)
    plot2.get_yaxis().get_major_formatter().set_useOffset(False)
    axes = plt.gca()
    axes.set_xlim([time_vec_np[0],time_vec_np[-1]])    
    plt.gcf().subplots_adjust(left=0.15)
    pp.savefig()

    #Alt vs time
    figureALT = plt.figure()
    plotALT = figureALT.add_subplot(1,1,1)
    plotALT.plot(time_vec_np,alt_vec_np)
    plotALT.set_xlabel('Time (HH:MM)')
    plotALT.set_ylabel('Altitude (m)')
    plotALT.set_xticks(xticks) #linspace - start,end,number of points
    plotALT.set_xticklabels(time_vec_HHMM,rotation=0,ha='right',fontsize=12)
    axes = plt.gca()
    axes.set_xlim([time_vec_np[0],time_vec_np[-1]])    
    plotALT.grid()
    pp.savefig()

    #Top Down View
    figure3 = plt.figure()
    plot3 = figure3.add_subplot(1,1,1)
    plot3.plot(lat_vec_np,lon_vec_np)
    plot3.set_xlabel('Latitude (N)')
    plot3.set_ylabel('Longitude (W)')
    plot3.grid()
    plot3.get_yaxis().get_major_formatter().set_useOffset(False)
    plot3.get_xaxis().get_major_formatter().set_useOffset(False)
    plt.gcf().subplots_adjust(left=0.15)
    pp.savefig()

    plt.figure()
    plt.plot(x_vec_np,y_vec_np,marker="s")
    plt.plot(x_vec_np[0],y_vec_np[0],marker="^")
    plt.plot(x_vec_np[-1],y_vec_np[-1],marker="v")
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid()
    pp.savefig()

    ##X and Y
    figureX = plt.figure()
    pltX = figureX.add_subplot(1,1,1)
    pltX.plot(time_vec_np,x_vec_np)
    pltX.set_xlabel('Time (HH:MM)')
    pltX.set_ylabel('X (m)')
    pltX.set_xticks(xticks) #linspace - start,end,number of points
    pltX.set_xticklabels(time_vec_HHMM,rotation=0,ha='right',fontsize=12)
    axes = plt.gca()
    axes.set_xlim([time_vec_np[0],time_vec_np[-1]])    
    pltX.grid()
    pp.savefig()

    figureY = plt.figure()
    pltY = figureY.add_subplot(1,1,1)
    pltY.plot(time_vec_np,y_vec_np)
    pltY.set_xlabel('Time (HH:MM)')
    pltY.set_ylabel('Y (m)')
    pltY.set_xticks(xticks) #linspace - start,end,number of points
    pltY.set_xticklabels(time_vec_HHMM,rotation=0,ha='right',fontsize=12)
    axes = plt.gca()
    axes.set_xlim([time_vec_np[0],time_vec_np[-1]])    
    pltY.grid()
    pp.savefig()

    #Run y through a derivative element
    # ydot_vec_np = [0]
    # xdot_vec_np = [0]
    # for idx in range(0,len(y_vec_np)-1):
    #     dt = time_vec_np[idx+1]-time_vec_np[idx]
    #     if dt != 0:
    #         ydot = (y_vec_np[idx+1]-y_vec_np[idx])/dt
    #         xdot = (x_vec_np[idx+1]-x_vec_np[idx])/dt
    #     ydot_vec_np.append(ydot)
    #     xdot_vec_np.append(xdot)

    # plt.figure()
    # plt.plot(time_vec_np,xdot_vec_np)
    # plt.xlabel('Time (Hr)')
    # plt.ylabel('Xdot (m/s)')
    # plt.grid()
    # pp.savefig()

    # plt.figure()
    # plt.plot(time_vec_np,ydot_vec_np)
    # plt.xlabel('Time (Hr)')
    # plt.ylabel('Ydot (m/s)')
    # plt.grid()
    # pp.savefig()

    # #CLOSE FILE
    if (openfile):
        pp.close()

        # #AND THEN USE EVINCE TO OPEN PDF if on linux
        if sys.platform == 'linux2':
            os.system('evince plots.pdf &')

    return pp

if __name__ == "__main__":

    filename = 'GPSLOG00.TXT'
    data = gps_data(filename)
    pp = PdfPages('plots.pdf')
    create_gps_plots(data,5,pp)

# Copyright - Carlos Montalvo 2016
# You may freely distribute this file but please keep my name in here
# as the original owner
