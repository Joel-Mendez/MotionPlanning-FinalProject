function TrajectoryType = getTrajectoryType(ObstacleHeight,ObstacleLength,ObstacleDistance,footLength,legLength,maxStrideLength)

if .5*footLength+ObstacleDistance+ObstacleLength+ObstacleHeight < maxStrideLength:
    TrajectoryType = 'WalkOver';
elseif ObstacleHeight < (legLength/2):
    TrajectoryType = 'StepOver';
else:
    TrajectoryType = 'Stand';

end

