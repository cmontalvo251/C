clear
clc
close all

%delete old
system('del a.exe')
system('del MATLAB.o')
system('del file')
system('del FinalProject.out')
%compile
system('g++ MATLAB.cpp -c -w')
system('g++ MATLAB.o NonlinearFinalProject.cpp -w')
%read data(name of out file output from C++)
fig1 = figure();
fig2 = figure();
fig3 = figure();
fig4 = figure();
fig5 = figure();
fig6 = figure();
fig7 = figure();


for ic1 = 10
    for ic2 = 6
        for ic3 = 3
            for ic4 = 9.6
                for ic5 = 5.5
                    for ic6 = 2.7
                        for ic7 = .2
                            for ic8 = .2
                                for ic9 = .2
                                    for ic10 = .2
                                        for ic11 = .2
                                            for ic12 = .2
                                                for ic13 = .2
                                                    command = ['a.exe',' ',num2str(ic1),' ',num2str(ic2),' ',num2str(ic3),' ',num2str(ic4),' ',num2str(ic5),' ',num2str(ic6),' ',num2str(ic7),' ',num2str(ic8),' ',num2str(ic9),' ',num2str(ic10),' ',num2str(ic11),' ',num2str(ic12),' ',num2str(ic13),' > file'];
                                                    system(command)
                                                    %read data
                                                    %plot data
                                                    data = dlmread('FinalProject.out');
                                                    
                                                    time = data(:,1);
                                                    q0= data(:,2);
                                                    q1= data(:,3);
                                                    q2= data(:,4);
                                                    q3= data(:,5);
                                                    p= data(:,6);
                                                    q= data(:,7);
                                                    r= data(:,8);
                                                    qm0= data(:,9);
                                                    qm1= data(:,10);
                                                    qm2= data(:,11);
                                                    qm3= data(:,12);
                                                    pm= data(:,13);
                                                    qm= data(:,14);
                                                    rm= data(:,15);
                                                    Pq0 = data(:,16);
                                                    Pq1 = data(:,17);
                                                    Pq2 = data(:,18);
                                                    Pq3 = data(:,19);
                                                    Pp = data(:,20);
                                                    Pq = data(:,21);
                                                    Pr = data(:,22);
                                                    
                                                    set(0,'CurrentFigure',fig1)
                                                    plot(time,q0,time,qm0,time,Pq0)
                                                    set(fig1,'color','white');
                                                    set(gca,'fontsize',18)
                                                    xlabel('Time(s)')
                                                    ylabel('q0')
                                                    title('q0, qm0, Pq0 vs. Time')
                                                    grid on
                                                    hold on
                                                    
                                                    set(0,'CurrentFigure',fig2)
                                                    plot(time,q1,time,qm1,time,Pq1)
                                                    set(fig2,'color','white');
                                                    set(gca,'fontsize',18)
                                                    xlabel('Time(s)')
                                                    ylabel('q1')
                                                    title('q1, qm1, Pq1 vs. Time')
                                                    grid on
                                                    hold on
                                                    
                                                    set(0,'CurrentFigure',fig3)
                                                    plot(time,q2,time,qm2,time,Pq2)
                                                    set(fig3,'color','white');
                                                    set(gca,'fontsize',18)
                                                    xlabel('Time(s)')
                                                    ylabel('q2')
                                                    title('q2, qm2, Pq2 vs. Time')
                                                    grid on
                                                    hold on
                                                    
                                                    set(0,'CurrentFigure',fig4)
                                                    plot(time,q3,time,qm3,time,Pq3)
                                                    set(fig4,'color','white');
                                                    set(gca,'fontsize',18)
                                                    xlabel('Time(s)')
                                                    ylabel('q3')
                                                    title('q3, qm3, Pq3 vs. Time')
                                                    grid on
                                                    hold on
                                                    
                                                    set(0,'CurrentFigure',fig5)
                                                    plot(time,p,time,pm,time,Pp)
                                                    set(fig5,'color','white');
                                                    set(gca,'fontsize',18)
                                                    xlabel('Time(s)')
                                                    ylabel('Angular Velocity')
                                                    title('Angular Velocity - p, pm, Pp vs. Time')
                                                    grid on
                                                    hold on
                                                    
                                                    set(0,'CurrentFigure',fig6)
                                                    plot(time,q,time,qm,time,Pq)
                                                    set(fig7,'color','white');
                                                    set(gca,'fontsize',18)
                                                    xlabel('Time(s)')
                                                    ylabel('Angular Velocity')
                                                    title('Angular Velocity - q, qm, Pq vs. Time')
                                                    grid on
                                                    hold on
                                                    
                                                    set(0,'CurrentFigure',fig7)
                                                    plot(time,r,time,rm,time,Pr)
                                                    set(fig7,'color','white');
                                                    set(gca,'fontsize',18)
                                                    xlabel('Time(s)')
                                                    ylabel('Angular Velocity')
                                                    title('Angular Velocity - r, rm, pr vs. Time')
                                                    grid on
                                                    hold on
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end