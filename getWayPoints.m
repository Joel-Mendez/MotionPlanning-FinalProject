function [LeadingWaypoints,TrailingWaypoints,Torso1Waypoints,Torso2Waypoints] = getWayPoints(clientID,TrajectoryType,ObstacleHandle,legLength)

%Getting Handles
[~,LeadingFootHandle] = vrep.simxGetObjectHandle(clientID,'Rectangle8',vrep_opmode_blocking);
[~,TrailingFootHandle] = vrep.simxGetObjectHandle(clientID,'Rectangle7',vrep_opmode_blocking);
[~,TorsoHandle] = vrep.simxGetObjectHandle(clientID,'Bill_rightLegJoint',vrep_opmode_blocking);
[~,FloorHanlde] = vrep.simxgetObjectHandle(clientID,'ResizableFloor_5_25',vrep_opmode_blocking);

%Getting Position of Torso, Feet, and Obstacle
LeadingFootPosition = vrep.simxGetObjectPosition(clientID,LeadingFootHandle,FloorHandle,vrep.simx_opmode_blocking);
TrailingFootPosition= vrep.simxGetObjectPosition(clientID,TrailingFootHandle,FloorHandle,vrep.simx_opmode_blocking);
TorsoPosition=vrep.simxGetObjectPosition(clientID,TorsoHandle,FloorHandle,vrep.simx_opmode_blocking);
ObstaclePosition = vrep.simxGetObjectPosition(clientID,ObstacleHandle,FloorHandle,vrep.simx_opmode_blocking);

%Breaking down positions into xyz components
LeadingFootX=LeadingFootPosition[1];
LeadingFootY=LeadingFootPosition[2];
LeadingFootZ=LeadingFootPosition[3];

TrailingFootX=TrailingFootPosition[1];
TrailingFootY=TrailingFootPosition[2];
TrailingFootZ=TrailingFootPosition[3];

TorsoX=TorsoPosition[1];
TorsoY=TorsoPosition[2];
TorsoZ=TorsoPosition[3];

ObstacleX=ObstaclePosition[1];
ObstacleZ=ObstaclePosition[3];


%Going into IF statements for different trajectory types.
if TrajectoryType == 'WalkOver'
  
    %Leading Foot Waypoints
    l1=[LeadingFootX,LeadingFootY,ObstacleZ+.5*ObstacleHeight+footHeight];
    l2=[ObstacleX,LeadingFootY,l1[3]];
    l3=[ObstacleX+.5*ObstacleLength+.5*footLength,LeadingFootY,l3[3]];
    l4=[l3[1],LeadingFootY,LeadingFootZ];
    l5=[l3[1],LeadingFootY,LeadingFootZ];
    l6=[l3[1],LeadingFootY,LeadingFootZ];
    l7=[l3[1],LeadingFootY,LeadingFootZ];
    l8=[l3[1],LeadingFootY,LeadingFootZ];
    
    %Trailing Foot Waypoints
    tr1=[TrailingFootX,TrailingFootY,TrailingFootZ];
    tr2=[TrailingFootX,TrailingFootY,TrailingFootZ];
    tr3=[TrailingFootX,TrailingFootY,TrailingFootZ];
    tr4=[TrailingFottX,TrailingFootY,TrailingFootZ+footHeight];
    tr5=[l1(1),TrailingFootY,l1(3)];
    tr6=[l2(1),TrailingFootY,l1(3)];
    tr7=[l3(1),TrailingFootY,l3(3)];
    tr8=[l4(1),TrailingFootY,l4(3)];
    
    %Torso Waypoints

    dx=(l8(1)-l1(1))/8;
    TX=[];
    TZ=[];
    for i = 1:1:8
        TX(i)=Torso1X+i*dx;
    end
    
    
    t1_1=[TX(1),TorsoY,tr1(3)+sqrt(legLength-(TX(1)-tr1(1))^2)];
    t1_2=[TX(2),TorsoY,tr1(3)+sqrt(legLength-(TX(2)-tr1(1))^2)];
    t1_3=[TX(3),TorsoY,tr1(3)+sqrt(legLength-(TX(3)-tr1(1))^2)];
    t1_4=[TX(4),TorsoY,tr1(3)+sqrt(legLength-(TX(4)-tr1(1))^2)];
    t1_5=[TX(5),TorsoY,l8(3)+sqrt(legLength-(l8(1)-TX(5))^2)];
    t1_6=[TX(6),TorsoY,l8(3)+sqrt(legLength-(l8(1)-TX(6))^2)];
    t1_7=[TX(7),TorsoY,l8(3)+sqrt(legLength-(l8(1)-TX(7))^2)];
    t1_8=[TX(8),TorsoY,TorsoZ];

    t2_1=t1_1;
    t2_2=t1_2;
    t2_3=t1_3;
    t2_4=t1_4;
    t2_5=t1_5;
    t2_6=t1_6;
    t2_7=t1_7;
    t2_8=t1_8;
    
    t2_1(2)=Torso2Y;
    t2_2(2)=Torso2Y;
    t2_3(2)=Torso2Y;
    t2_4(2)=Torso2Y;
    t2_5(2)=Torso2Y;
    t2_6(2)=Torso2Y;
    t2_7(2)=Torso2Y;
    t2_8(2)=Torso2Y;

    
    LeadingWaypoints = [l1;l2;l3;l4;l5;l6;l7;l8];     
    TrailingWaypoints = [tr1;tr2;tr3;tr4;tr5;tr6;tr7;tr8]
    Torso1Waypoints = [t1_1;t1_2;t1_3;t1_4;t1_5;t1_6;t1_7;t1_8];
    Torso2Waypoints = [t2_1;t2_2;t2_3;t2_4;t2_5;t2_6;t2_7;t2_8];
                  

    
elseif TrajectoryType == 'StepOn':
    
    display('In Progress')
%     LeadingWaypoints=[leadPoint1;
%                       leadPoint2;
%                       leadPoint3;
%                       leadPoint4;
%                       leadPoint5;
%                       leadPoint6;
%                       leadPoint7;
%                       leadPoint8];
%                   
%     TrailingWaypoints=[trailPoint1;
%                       trailPoint2;
%                       trailPoint3;
%                       trailPoint4;
%                       trailPoint5;
%                       trailPoint6;
%                       trailPoint7;
%                       trailPoint8];
%                   
%     TorsoWaypoints=[torsoPoint1;
%                     torsoPoint2;
%                     torsoPoint3;
%                     torsoPoint4;
%                     torsoPoint5;
%                     torsoPoint6;
%                     torsoPoint7;
%                     torsoPoint8];
    
    
else:
    LeadingWaypoints=[];
    TrailingWaypoints=[];
    TorsoWaypoints=[];
    
    for i = 1:1:8
    LeadingWaypoints=[LeadingWaypoints;
                        LeadingFootX,LeadingFootY,LeadingFootZ];
    TrailingWaypoints=[LeadingWaypoints;
                        TrailingFootX,TrailingFootY,TrailingFootZ];
    TorsoWaypoints=[LeadingWaypoints;
                        TorsoX,TorsoY,TorsoZ];
    end
end
    


end

