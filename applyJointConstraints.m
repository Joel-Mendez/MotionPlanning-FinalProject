function [ThighAngle,KneeAngle,AnkleAngle] = applyJointConstraints(ThighAngle,KneeAngle,AnkleAngle)

%Get reasonable values from team
%Check how the angles work with Vrep
minThighAngle = -20
maxThighAngle = 90
minKneeAngle = 90
maxKneeAngle = 0
minAnkleAngle = -20
maxAnkleAngle = 20


if ThighAngle < minThighAngle:
    ThighAngle = minThighAngle;
elseif ThighAngle > maxThighAngle:
    ThighAngle = maxThighAngle;
    
if KneeAngle < minKneeAngle:
    KneeAngle = minKneeAngle;
elseif KneeAngle > maxKneeAngle:
    KneeAngle = maxKneeAngle;
    
if AnkleAngle < minAnkleAngle:
    AnkleAngle = minAnkleAngle;
elseif AnkleAngle > maxAnkleAngle:
    AnkleAngle = maxAnkleAngle;

end

