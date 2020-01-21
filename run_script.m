clear
clc
close all

system('rm a.out')  %%remove
system('g++ rk4.cpp') %%compile
system('./a.out') %%%run

data = dlmread('SimulationResultsC++.out'); %%%read in data file

time = data(:,1);
x = data(:,2);
xdot = data(:,3);

plot(time,x)