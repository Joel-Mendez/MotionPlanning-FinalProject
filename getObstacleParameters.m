function ObstacleParameters = getObstacleParameters(clientID,ObstacleHandle)

[~,zmax]=vrep.simxGetObjectFloatParameter(clientID,ObstacleHandle,20,vrep.simx_opmode_blocking); %Not sure abot the opmode
[~,zmin]=vrep.simxGetObjectFloatParameter(clientID,ObstacleHandle,17,vrep.simx_opmode_blocking);
[~,xmax]=vrep.simxGetObjectFloatParameter(clientID,ObstacleHandle,18,vrep.simx_opmode_blocking);
[~,xmin]=vrep.simxGetObjectFloatParameter(clientID,ObstacleHandle,15,vrep.simx_opmode_blocking);

ObstacleHeight=zmax-zmin;
ObstacleLength=xmax-xmin;
ObstacleParameters = [ObstacleHeight, ObstacleLength];

end

