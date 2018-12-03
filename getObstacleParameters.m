function ObstacleParameters = getObstacleParameters(clientID,ObstacleHandle)

[~,zmax]=simxGetObjectFloatParameter(clientID,ObstacleHandle,20,vrep.simx_opmode_blocking); %Not sure abot the opmode
[~,zmin]=simxGetObjectFloatParameter(clientID,ObstacleHandle,17,vrep.simx_opmode_blocking);
[~,xmax]=simxGetObjectFloatParameter(clientID,ObstacleHandle,18,vrep.simx_opmode_blocking);
[~,xmin]=simxGetObjectFloatParameter(clientID,ObstacleHandle,15,vrep.simx_opmode_blocking);

ObstacleHeight=zmax-zmin;
ObstacleLength=xmax-xmin;
ObstacleParameters = [ObstacleHeight, ObstacleLength];

end

