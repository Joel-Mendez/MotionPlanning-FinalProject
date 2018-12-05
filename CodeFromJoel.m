%Portion that I wrote in VRep 
%Does not run by itself. 

%% This section should go near the top where you define the Object and Sensor Handles. 

%I believe you already have the Joint Handles taken care of. 
% legJointHandles={sim.getObjectHandle('Bill_leftLegJoint'),sim.getObjectHandle('Bill_rightLegJoint')};
% kneeJointHandles={sim.getObjectHandle('Bill_leftKneeJoint'),sim.getObjectHandle('Bill_rightKneeJoint')};
% ankleJointHandles={sim.getObjectHandle('Bill_leftAnkleJoint'),sim.getObjectHandle('Bill_rightAnkleJoint')};

%Sensors that I added to Bill's feet. See BillEnv.ttt in GitHub
[~,RightSensorHandle]=vrep.simxGetObjectHandle(clientID,'RightFootSensor',vrep_simx_opmode_blocking);
[~,LeftSensorHandle]=vrep.simxGetObjectHandle(clientID,'LeftFootSensor',vrep_simx_opmode_blocking);

%Defining Leg Parameters
thighLength=.9078-.5147; %Read from Screen:(zposition of leg joint) - (zposition of knee joint) CHANGE IF WE'RE NOT USING BILL!!!
shankLength=.5147-.1038; %Read from Screen:(zposition of knee joint) - (zposition of ankle joint) CHANGE IF WE'RE NOT USING BILL!!!
footLength=.8906; %Read from Group 16 geometry - Size-X CHANGE IF WE'RE NOT USING BILL!!!
footHeight=.0198; %zCoordinate from AnkleJoint position CHANGE IF WE'RE NOT USING BILL!!!

legLength=thighLength+shankLength+footHeight
maxStrideLength=1.41*legLength 

%% This Section should go at the start of your Actuation while loop. 

%Checking Sensors
%NOTE: TAKE CARE OF THIS BEFORE SENDING IT TO MARSHALL
ObstacleDetected,ObstacleDistance,DetectedPoint,ObstacleHandle,SurfaceNormVec=sim.handleProximitySensor(sim.handle_all);

if ObstacleDetected==1: %Obstacle Detected
    
    %Retrieving Obstacle Parameters
    ObstacleParameters=getObstacleParameters(clientID,ObstacleHandle);
    ObstacleHeight = ObstacleParameters[1];
    ObstacleLength = ObstacleParameters[2];
    
    %Determining Trajectory Type from Obstacle Parameters
    TrajectoryType = getTrajectoryType(ObstacleHeight,ObstacleLength,ObstacleDistance,footLength,legLength,maxStrideLength);
    
    %Create Waypoints depending on the TrajectoryType.
    [LeadingWaypoints,TrailingWaypoints,TorsoWaypoints] = getWayPoints(clientID,TrajectoryType, ObstacleHandle)
    %Each set of Waypoints will be a 8x3 matrix. row=Waypoint, column=x,y,z
    
    %Find Joint Angles at each waypoint
    
%     %Pseudo-Code
%     JointAngles=[];
%     for i = 1:1:8 %8 waypoints
%         angles=invkine(LeadingWaypoint(i,:),TrailingWaypoint(i,:),TorsoWaypoint(i,:));% 
%         JointAngles=[JointAngles;angles];
%     end


    %Create Trajectory from Waypoint to Waypoint using minjerk and Joint
    %Angles
    
%     %Pseudo-Code
%     Position=[] %Will end up being a 6xn matrix, which each row corresponding to a joint
%     for angles in JointAngles:
%         PositionCoefficients = flip(minjerk(InitialValues,angles,T)
%         %Find Position values using the resulting polynomial. 
%         %Populate Position: Position = [Position, newPositionValues]
    
    %move according to trajectoy
    
else
    %Bill moves as usual
end



