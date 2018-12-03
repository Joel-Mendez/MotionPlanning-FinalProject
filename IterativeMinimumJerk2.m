%Edited from the one created with Dr. Lenzi
%Using a while loop so that I don't iterate over Time and can observe how
%the trajectory changes as Tf changes
clear vars
close all


%Initial Values: x0=Position, dx0=Velocity, ddx0=Acceleration
x0= 60; %Initial Position
dx0=0; %Initial Velocity
ddx0= 0; %Initial Acceleration

%Changing Variables used for Iterative Process
x0_old= 60; 
dx0_old=0;  
ddx0_old= 0;

%Desired Final Values
xT= 0;
dxT=0;
ddxT=0;

%Desired Time
T=2;
Tf= 2; %Time used for Iterative Process
timestep=.1; 
N = 1000; %Number of steps
dt = 0.002; 
time=linspace(0,T,T/dt);

%Original trajectory
Traj0=minimumJerk(x0,dx0,ddx0,xT,dxT,ddxT,T);
Traj0=flip(Traj0);
figure(1), hold all
plot(time, polyval(Traj0,time))

reached = 0;
tplot=0;
%for i = 1:length(time)
while reached ==0

    p = minimumJerk(x0_old,dx0_old,ddx0_old,xT,dxT,ddxT,Tf);
    v = [p(2);2*p(3);3*p(4);4*p(5);5*p(6)];
    a = [2*p(3);6*p(4);12*p(5);20*p(6)];
    
    p = flip(p); %Position Coefficients
    v= flip(v);
    a =flip (a);
    x0_old = polyval(p, dt); %Position Equation
    dx0_old =  polyval(v, dt); %Velocity Equation
    ddx0_old = polyval(a, dt); %Acceleration Equation
  
%     figure(1),
    %plot(time(i),x0_old,'o-')
    plot(tplot,x0_old,'o-')
    title('Minimum Jerk Trajectory')
    xlabel('Time (s)')
    ylabel('Knee Angle (deg.)')
    if tplot>1
        xT = 20;
        T = 2; %+ 1*sin(2*pi*time(i));
%         figure(101), hold all, plot(T,'.')
    end
    
    tplot=tplot+dt;
    %Tf = T-time(i);
    Tf=T-tplot;
%     pause
    
    if tplot>5
        reached =1;
    end

    if x0_old==xT
        reached = 1;
    end
    
    
    
    
end
xlim([0 T])



