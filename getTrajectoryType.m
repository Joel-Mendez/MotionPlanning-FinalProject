function TrajectoryType = getTrajectoryType(ObstacleParameters,ObstacleDistance,footLength,legLength,maxStrideLength)

ObstacleHeigth = ObstacleParameters[1];
ObstacleLength = ObstacleParameters[2];

if .5*footLength+ObstacleDistance+ObstacleLength+ObstacleHeight < maxStrideLength:
    TrajectoryType = 'WalkOver';
elseif ObstacleHeight < (legLength/2):
    TrajectoryType = 'StepOver';
else:
    TrajectoryType = 'Stand';

end

