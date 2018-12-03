disp('Program started');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m) - You can find it in C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\matlab\matlab if you've downloaded V-REP
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID > -1)
    disp('Connected to remote API server');
    
    % Get model handle
    %[~,modelHandle]=vrep.simxGetObjectAssociatedWithScript(clientID,vrep.simxHandleSelf,vrep.simx_opmode_blocking);
    modelHandle=16;
    
    %Get object handles%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Get Handles for Joints
    [~,llj]=vrep.simxGetObjectHandle(clientID,'Bill_leftLegJoint',vrep.simx_opmode_blocking);
    [~,rlj]=vrep.simxGetObjectHandle(clientID,'Bill_rightLegJoint',vrep.simx_opmode_blocking);
    legJointHandles=[llj,rlj];
    [~,lkj]=vrep.simxGetObjectHandle(clientID,'Bill_leftKneeJoint',vrep.simx_opmode_blocking);
    [~,rkj]=vrep.simxGetObjectHandle(clientID,'Bill_rightKneeJoint',vrep.simx_opmode_blocking);
    kneeJointHandles=[lkj,rkj];
    [~,laj]=vrep.simxGetObjectHandle(clientID,'Bill_leftAnkleJoint',vrep.simx_opmode_blocking);
    [~,raj]=vrep.simxGetObjectHandle(clientID,'Bill_rightAnkleJoint',vrep.simx_opmode_blocking);
    ankleJointHandles=[laj,raj];
    [~,lsj]=vrep.simxGetObjectHandle(clientID,'Bill_leftShoulderJoint',vrep.simx_opmode_blocking);
    [~,rsj]=vrep.simxGetObjectHandle(clientID,'Bill_rightShoulderJoint',vrep.simx_opmode_blocking);
    shoulderJointHandles=[lsj,rsj];
    [~,lej]=vrep.simxGetObjectHandle(clientID,'Bill_leftElbowJoint',vrep.simx_opmode_blocking);
    [~,rej]=vrep.simxGetObjectHandle(clientID,'Bill_rightElbowJoint',vrep.simx_opmode_blocking);
    elbowJointHandles=[lej,rej];
    [~,neckJoint]=vrep.simxGetObjectHandle(clientID,'Bill_neck',vrep.simx_opmode_blocking);
    
    %Get Handles for Sensors
    [~,leftSensorClose]=vrep.simxGetObjectHandle(clientID,'Bill_proxSensorLeftClose',vrep.simx_opmode_blocking);
    [~,rightSensorClose]=vrep.simxGetObjectHandle(clientID,'Bill_proxSensorRightClose',vrep.simx_opmode_blocking);
    [~,leftSensorFar]=vrep.simxGetObjectHandle(clientID,'Bill_proxSensorLeftFar',vrep.simx_opmode_blocking);
    [~,rightSensorFar]=vrep.simxGetObjectHandle(clientID,'Bill_proxSensorRightFar',vrep.simx_opmode_blocking);
    [~,leftFloorSensorClose]=vrep.simxGetObjectHandle(clientID,'Bill_proxSensorFloorLeftClose',vrep.simx_opmode_blocking);
    [~,rightFloorSensorClose]=vrep.simxGetObjectHandle(clientID,'Bill_proxSensorFloorRightClose',vrep.simx_opmode_blocking);
    [~,leftFloorSensorFar]=vrep.simxGetObjectHandle(clientID,'Bill_proxSensorFloorLeftFar',vrep.simx_opmode_blocking);
    [~,rightFloorSensorFar]=vrep.simxGetObjectHandle(clientID,'Bill_proxSensorFloorRightFar',vrep.simx_opmode_blocking);
    
    % Define walking waypoints
    legWaypoints=[0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450,-0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,0.145,0.188];
    kneeWaypoints=[0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243,0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092];
    ankleWaypoints=[-0.133,0.041,0.244,0.382,0.304,0.232,0.266,0.061,-0.090,-0.145,-0.043,0.041,0.001,0.011,-0.099,-0.127,-0.121,-0.120,-0.107,-0.100,-0.090,-0.009];
    shoulderWaypoints=[0.028,0.043,0.064,0.078,0.091,0.102,0.170,0.245,0.317,0.337,0.402,0.375,0.331,0.262,0.188,0.102,0.094,0.086,0.080,0.051,0.058,0.048];
    elbowWaypoints=[-1.148,-1.080,-1.047,-0.654,-0.517,-0.366,-0.242,-0.117,-0.078,-0.058,-0.031,-0.001,-0.009,0.008,-0.108,-0.131,-0.256,-0.547,-0.709,-0.813,-1.014,-1.102];
    relativeVel=[2,2,1.2,2.3,1.4,1,1,1,1,1.6,1.9,2.4,2.0,1.9,1.5,1,1,1,1,1,2.3,1.5];
    
    % Define nominal walking velocity
    nominalVelocity=0.5;
    speedModulator=1;
    
    % Define constants
    vel=nominalVelocity*0.8/0.56;
    scaling=0;
    tl=length(legWaypoints);
    dl=1/tl;
    vp=0;
    floorNotDetected=0;
    obstacleDetected=0;
    movement=0;
    floorMovement=0;
    obstacleMovement=0;
    
    % Initialize sensor readings
    allFloorSensorsTriggerCount=0;
    allFloorSensorsTriggerCountRequired=5;
    obstacleSensorsNoTriggerCount=0;
    obstacleSensorsNoTriggerCountRequired=2;
    
    %dt=sim.getSimulationTimeStep()
    %simTime=sim.getSimulationTime()
    %dt=vrep.simxGetSimulationTimeStep();
    %simTime = vrep.simxGetSimulationTime();
    dt=.05;
    
    % Initialize sensor readings
    [~,lfc]=vrep.simxReadProximitySensor(clientID,leftFloorSensorClose,vrep.simx_opmode_streaming);
    [~,rfc]=vrep.simxReadProximitySensor(clientID,rightFloorSensorClose,vrep.simx_opmode_streaming);
    [~,lff]=vrep.simxReadProximitySensor(clientID,leftFloorSensorFar,vrep.simx_opmode_streaming);
    [~,rff]=vrep.simxReadProximitySensor(clientID,rightFloorSensorFar,vrep.simx_opmode_streaming);
    [~,lc,lcd]=vrep.simxReadProximitySensor(clientID,leftSensorClose,vrep.simx_opmode_streaming);
    [~,rc,rcd]=vrep.simxReadProximitySensor(clientID,rightSensorClose,vrep.simx_opmode_streaming);
    [~,lf,lfd]=vrep.simxReadProximitySensor(clientID,leftSensorFar,vrep.simx_opmode_streaming);
    [~,rf,rfd]=vrep.simxReadProximitySensor(clientID,rightSensorFar,vrep.simx_opmode_streaming);
    
    % Initialize joint readings
    [~,lhp]=vrep.simxGetJointPosition(clientID,legJointHandles(1),vrep.simx_opmode_streaming);
    [~,lkp]=vrep.simxGetJointPosition(clientID,kneeJointHandles(1),vrep.simx_opmode_streaming);
    [~,lap]=vrep.simxGetJointPosition(clientID,ankleJointHandles(1),vrep.simx_opmode_streaming);
    [~,rhp]=vrep.simxGetJointPosition(clientID,legJointHandles(2),vrep.simx_opmode_streaming);
    [~,rkp]=vrep.simxGetJointPosition(clientID,kneeJointHandles(2),vrep.simx_opmode_streaming);
    [~,rap]=vrep.simxGetJointPosition(clientID,ankleJointHandles(2),vrep.simx_opmode_streaming);
    lhv=0; lha=0;
    rhv=0; rha=0;
    lkv=0; lka=0;
    rkv=0; rka=0;
    lav=0; laa=0;
    rav=0; raa=0;
    
    while(1)
        % 1. First check what the floor sensors tell us:
        if (floorNotDetected==0)
            % According to the floor sensors, we should walk straight here
            % We have to check the close sensors to know whether we wanna turn (according to the floor sensors)
            [~,lfc]=vrep.simxReadProximitySensor(clientID,leftFloorSensorClose,vrep.simx_opmode_buffer);
            [~,rfc]=vrep.simxReadProximitySensor(clientID,rightFloorSensorClose,vrep.simx_opmode_buffer);
            if (lfc<1)&&(rfc<1)
                if (randn(1)>0.5)
                    lfc=1;
                else
                    rfc=1;
                end
            end
            if (lfc<1)
                floorMovement=1;
                floorNotDetected=floorMovement;
            end
            if (rfc<1)
                floorMovement=-1;
                floorNotDetected=floorMovement;
            end
        else
            % According to the floor sensors, we should rotate here
            % We have to check the far sensors to know whether we wanna walk straight again (according to the floor sensors)
            [~,lff]=vrep.simxReadProximitySensor(clientID,leftFloorSensorFar,vrep.simx_opmode_buffer);
            [~,rff]=vrep.simxReadProximitySensor(clientID,rightFloorSensorFar,vrep.simx_opmode_buffer);
            if (lff>0)&&(rff>0)
                allFloorSensorsTriggerCount=allFloorSensorsTriggerCount+1;
            end
            if (allFloorSensorsTriggerCount*dt/0.05>allFloorSensorsTriggerCountRequired)
                disp('Floor not detected.')
                floorNotDetected=0;
                floorMovement=0;
                allFloorSensorsTriggerCount=0;
            end
        end
        
        % 2. Now check what the obstacle sensors tell us:
        [~,lc,lcd]=vrep.simxReadProximitySensor(clientID,leftSensorClose,vrep.simx_opmode_buffer);
        [~,rc,rcd]=vrep.simxReadProximitySensor(clientID,rightSensorClose,vrep.simx_opmode_buffer);
        [~,lf,lfd]=vrep.simxReadProximitySensor(clientID,leftSensorFar,vrep.simx_opmode_buffer);
        [~,rf,rfd]=vrep.simxReadProximitySensor(clientID,rightSensorFar,vrep.simx_opmode_buffer);
        if (obstacleDetected==0)
            % According to the floor sensors, we should walk straight here
            % We have to check for obstacles to know whether we wanna turn (according to the obstacle sensors)
            if (lc>0)&&(rc>0)
                if (lcd>rcd)
                    lc=0;
                else
                    rc=0;
                end
            end
            if (lc>0)
                obstacleMovement=1;
                obstacleDetected=obstacleMovement;
            end
            if (rc>0)
                obstacleMovement=-1;
                obstacleDetected=obstacleMovement;
            end
        else
            % According to the obstacle sensors, we should rotate here
            % We have to check whether we can walk straight again (according to the obstacle sensors)
            if (lf<1)&&(rf<1)
                obstacleSensorsNoTriggerCount=obstacleSensorsNoTriggerCount+1;
            end
            if (obstacleSensorsNoTriggerCount*dt/0.05>obstacleSensorsNoTriggerCountRequired)
                disp('Obstacle detected.')
                obstacleDetected=0;
                obstacleMovement=0;
                obstacleSensorsNoTriggerCount=0;
            end
        end
        
        %3. Now decide how to walk by evaluating what the floor tells us and what the obstacles tell us (the floor has more weight than obstacles)
        wantRotation=(floorMovement~=0)||(obstacleMovement~=0);
        if (wantRotation)
            if (movement==0)
                movement=floorMovement;
                if (movement==0)
                    movement=obstacleMovement;
                end
            end
        else
            movement=0;
        end
        
        %s=sim.getObjectSizeFactor(modelHandle)
        s=1;
        
        % Define walking waypoints
        legWaypoints=[0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450,-0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,0.145,0.188];
        kneeWaypoints=[0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243,0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092];
        ankleWaypoints=[-0.133,0.041,0.244,0.382,0.304,0.232,0.266,0.061,-0.090,-0.145,-0.043,0.041,0.001,0.011,-0.099,-0.127,-0.121,-0.120,-0.107,-0.100,-0.090,-0.009];
        shoulderWaypoints=[0.028,0.043,0.064,0.078,0.091,0.102,0.170,0.245,0.317,0.337,0.402,0.375,0.331,0.262,0.188,0.102,0.094,0.086,0.080,0.051,0.058,0.048];
        elbowWaypoints=[-1.148,-1.080,-1.047,-0.654,-0.517,-0.366,-0.242,-0.117,-0.078,-0.058,-0.031,-0.001,-0.009,0.008,-0.108,-0.131,-0.256,-0.547,-0.709,-0.813,-1.014,-1.102];
        relativeVel=[2,2,1.2,2.3,1.4,1,1,1,1,1.6,1.9,2.4,2.0,1.9,1.5,1,1,1,1,1,2.3,1.5];
        
        if (movement==0)
            scaling=1;
            vp=vp+dt*vel;
            p=mod(vp,1);
            indexLow=floor(p/dl);
            t=p/dl-indexLow;
            oppIndexLow=floor(indexLow+tl/2);
            if (oppIndexLow>=tl)
                oppIndexLow=oppIndexLow-tl;
            end
            indexHigh=indexLow+1;
            if (indexHigh>=tl)
                indexHigh=indexHigh-tl;
            end
            oppIndexHigh=oppIndexLow+1;
            if (oppIndexHigh>=tl)
                oppIndexHigh=oppIndexHigh-tl;
            end
            
            leftLegJointValue=(legWaypoints(indexLow+1)*(1-t)+legWaypoints(indexHigh+1)*t)*scaling;
            leftKneeJointValue=(kneeWaypoints(indexLow+1)*(1-t)+kneeWaypoints(indexHigh+1)*t)*scaling;
            leftAnkleJointValue=(ankleWaypoints(indexLow+1)*(1-t)+ankleWaypoints(indexHigh+1)*t)*scaling;
            leftShoulderJointValue=(shoulderWaypoints(indexLow+1)*(1-t)+shoulderWaypoints(indexHigh+1)*t)*scaling;
            leftElbowJointValue=(elbowWaypoints(indexLow+1)*(1-t)+elbowWaypoints(indexHigh+1)*t)*scaling;
            
            rightLegJointValue=(legWaypoints(oppIndexLow+1)*(1-t)+legWaypoints(oppIndexHigh+1)*t)*scaling;
            rightKneeJointValue=(kneeWaypoints(oppIndexLow+1)*(1-t)+kneeWaypoints(oppIndexHigh+1)*t)*scaling;
            rightAnkleJointValue=(ankleWaypoints(oppIndexLow+1)*(1-t)+ankleWaypoints(oppIndexHigh+1)*t)*scaling;
            rightShoulderJointValue=(shoulderWaypoints(oppIndexLow+1)*(1-t)+shoulderWaypoints(oppIndexHigh+1)*t)*scaling;
            rightElbowJointValue=(elbowWaypoints(oppIndexLow+1)*(1-t)+elbowWaypoints(oppIndexHigh+1)*t)*scaling;
            
            
            vvv=s*nominalVelocity*scaling*(relativeVel(indexLow+1)*(1-t)+relativeVel(indexHigh+1)*t);
            
            [~,m]=vrep.simxGetObjectOrientation(clientID,modelHandle,-1,vrep.simx_opmode_blocking);
            [~,r]=vrep.simxGetObjectPosition(clientID,modelHandle,-1,vrep.simx_opmode_blocking);
            T=genTmat(m,r);
            r(1)=r(1)+T(1)*dt*vvv;
            r(2)=r(2)+T(5)*dt*vvv;
            vrep.simxSetObjectPosition(clientID,modelHandle,-1,r,vrep.simx_opmode_blocking);
        else
            % Calculate the velocities and accelerations of the joints
            lhp_prev=lhp; lhv_prev=lhv;
            rhp_prev=rhp; rhv_prev=rhv;
            lkp_prev=lkp; lkv_prev=lkv;
            rkp_prev=rkp; rkv_prev=rkv;
            lap_prev=lap; lav_prev=lav;
            rap_prev=rap; rav_prev=rav;
            [~,lhp]=vrep.simxGetJointPosition(clientID,legJointHandles(1),vrep.simx_opmode_buffer);
            [~,rhp]=vrep.simxGetJointPosition(clientID,legJointHandles(2),vrep.simx_opmode_buffer);
            [~,lkp]=vrep.simxGetJointPosition(clientID,kneeJointHandles(1),vrep.simx_opmode_buffer);
            [~,rkp]=vrep.simxGetJointPosition(clientID,kneeJointHandles(2),vrep.simx_opmode_buffer);
            [~,lap]=vrep.simxGetJointPosition(clientID,ankleJointHandles(1),vrep.simx_opmode_buffer);
            [~,rap]=vrep.simxGetJointPosition(clientID,ankleJointHandles(2),vrep.simx_opmode_buffer);
            lhv=(lhp-lhp_prev)/dt; lha=(lhv-lhv_prev)/dt;
            rhv=(rhp-rhp_prev)/dt; rha=(rhv-rhv_prev)/dt;
            lkv=(lkp-lkp_prev)/dt; lka=(lkv-lkv_prev)/dt;
            rkv=(rkp-rkp_prev)/dt; rka=(rkv-rkv_prev)/dt;
            lav=(lap-lap_prev)/dt; laa=(lav-lav_prev)/dt;
            rav=(rap-rap_prev)/dt; raa=(rav-rav_prev)/dt;
            
            % Set the desired swing time
            swing_time = 1.5;
            
            % Calculate the minimum jerk coefficients
            lh_coeffs = flip(minimumJerk(lhp,0,0,-pi/4,0,0,swing_time));
            rh_coeffs = flip(minimumJerk(rhp,0,0,0,0,0,swing_time));
            lk_coeffs = flip(minimumJerk(lkp,0,0,pi/4,0,0,swing_time));
            rk_coeffs = flip(minimumJerk(rkp,0,0,0,0,0,swing_time));
            la_coeffs = flip(minimumJerk(lap,0,0,-pi/10,0,0,swing_time));
            ra_coeffs = flip(minimumJerk(rap,0,0,0,0,0,swing_time));
            
            % Create the time array
            time_array=linspace(0,swing_time,tl);
            
            % Select the leg moving through minimum jerk
            leftLeg = 1;
            
            % Evaluate the polynomial expression to get the trajectory
            if leftLeg==1
                legWaypoints = polyval(lh_coeffs,time_array);
                kneeWaypoints = polyval(lk_coeffs,time_array);
                ankleWaypoints = polyval(la_coeffs,time_array);
                clear figure
                figure(1)
                plot(time_array,legWaypoints);
                hold on;
                plot(time_array,kneeWaypoints);
                plot(time_array,ankleWaypoints);
                hold off;
                scaling=1;
                vp=vp+dt*vel;
                p=mod(vp,1);
                indexLow=floor(p/dl);
                t=p/dl-indexLow;
                oppIndexLow=floor(indexLow+tl/2);
                if (oppIndexLow>=tl)
                    oppIndexLow=oppIndexLow-tl;
                end
                indexHigh=indexLow+1;
                if (indexHigh>=tl)
                    indexHigh=indexHigh-tl;
                end
                oppIndexHigh=oppIndexLow+1;
                if (oppIndexHigh>=tl)
                    oppIndexHigh=oppIndexHigh-tl;
                end
                leftLegJointValue=(legWaypoints(indexLow+1)*(1-t)+legWaypoints(indexHigh+1)*t)*scaling;
                leftKneeJointValue=(kneeWaypoints(indexLow+1)*(1-t)+kneeWaypoints(indexHigh+1)*t)*scaling;
                leftAnkleJointValue=(ankleWaypoints(indexLow+1)*(1-t)+ankleWaypoints(indexHigh+1)*t)*scaling;
                leftShoulderJointValue=(shoulderWaypoints(indexLow+1)*(1-t)+shoulderWaypoints(indexHigh+1)*t)*scaling;
                leftElbowJointValue=(elbowWaypoints(indexLow+1)*(1-t)+elbowWaypoints(indexHigh+1)*t)*scaling;
                
                legWaypoints = legWaypoints*0;
                kneeWaypoints = kneeWaypoints*0;
                ankleWaypoints = ankleWaypoints*0;
                
                rightLegJointValue=(legWaypoints(oppIndexLow+1)*(1-t)+legWaypoints(oppIndexHigh+1)*t)*scaling*0;
                rightKneeJointValue=(kneeWaypoints(oppIndexLow+1)*(1-t)+kneeWaypoints(oppIndexHigh+1)*t)*scaling*0;
                rightAnkleJointValue=(ankleWaypoints(oppIndexLow+1)*(1-t)+ankleWaypoints(oppIndexHigh+1)*t)*scaling*0;
                rightShoulderJointValue=(shoulderWaypoints(oppIndexLow+1)*(1-t)+shoulderWaypoints(oppIndexHigh+1)*t)*scaling;
                rightElbowJointValue=(elbowWaypoints(oppIndexLow+1)*(1-t)+elbowWaypoints(oppIndexHigh+1)*t)*scaling;
                
                vvv=s*nominalVelocity*scaling*(relativeVel(indexLow+1)*(1-t)+relativeVel(indexHigh+1)*t)*0.4;
                
                [~,m]=vrep.simxGetObjectOrientation(clientID,modelHandle,-1,vrep.simx_opmode_blocking);
                [~,r]=vrep.simxGetObjectPosition(clientID,modelHandle,-1,vrep.simx_opmode_blocking);
                T=genTmat(m,r);
                r(1)=r(1)+T(1)*dt*vvv;
                r(2)=r(2)+T(5)*dt*vvv;
                vrep.simxSetObjectPosition(clientID,modelHandle,-1,r,vrep.simx_opmode_blocking);
            else
                legWaypoints = polyval(rh_coeffs,time_array);
                kneeWaypoints = polyval(rk_coeffs,time_array);
                ankleWaypoints = polyval(ra_coeffs,time_array);
                
                scaling=1;
                vp=vp+dt*vel;
                p=mod(vp,1);
                indexLow=floor(p/dl);
                t=p/dl-indexLow;
                oppIndexLow=floor(indexLow+tl/2);
                if (oppIndexLow>=tl)
                    oppIndexLow=oppIndexLow-tl;
                end
                indexHigh=indexLow+1;
                if (indexHigh>=tl)
                    indexHigh=indexHigh-tl;
                end
                oppIndexHigh=oppIndexLow+1;
                if (oppIndexHigh>=tl)
                    oppIndexHigh=oppIndexHigh-tl;
                end
                rightLegJointValue=(legWaypoints(indexLow+1)*(1-t)+legWaypoints(indexHigh+1)*t)*scaling;
                rightKneeJointValue=(kneeWaypoints(indexLow+1)*(1-t)+kneeWaypoints(indexHigh+1)*t)*scaling;
                rightAnkleJointValue=(ankleWaypoints(indexLow+1)*(1-t)+ankleWaypoints(indexHigh+1)*t)*scaling;
                rightShoulderJointValue=(shoulderWaypoints(indexLow+1)*(1-t)+shoulderWaypoints(indexHigh+1)*t)*scaling;
                rightElbowJointValue=(elbowWaypoints(indexLow+1)*(1-t)+elbowWaypoints(indexHigh+1)*t)*scaling;
                
                legWaypoints = legWaypoints*0;
                kneeWaypoints = kneeWaypoints*0;
                ankleWaypoints = ankleWaypoints*0;
                
                leftLegJointValue=(legWaypoints(oppIndexLow+1)*(1-t)+legWaypoints(oppIndexHigh+1)*t)*scaling*0;
                leftKneeJointValue=(kneeWaypoints(oppIndexLow+1)*(1-t)+kneeWaypoints(oppIndexHigh+1)*t)*scaling*0;
                leftAnkleJointValue=(ankleWaypoints(oppIndexLow+1)*(1-t)+ankleWaypoints(oppIndexHigh+1)*t)*scaling*0;
                leftShoulderJointValue=(shoulderWaypoints(oppIndexLow+1)*(1-t)+shoulderWaypoints(oppIndexHigh+1)*t)*scaling;
                leftElbowJointValue=(elbowWaypoints(oppIndexLow+1)*(1-t)+elbowWaypoints(oppIndexHigh+1)*t)*scaling;
                
                vvv=s*nominalVelocity*scaling*(relativeVel(indexLow+1)*(1-t)+relativeVel(indexHigh+1)*t)*0.4;
                
                [~,m]=vrep.simxGetObjectOrientation(clientID,modelHandle,-1,vrep.simx_opmode_blocking);
                [~,r]=vrep.simxGetObjectPosition(clientID,modelHandle,-1,vrep.simx_opmode_blocking);
                T=genTmat(m,r);
                r(1)=r(1)+T(1)*dt*vvv;
                r(2)=r(2)+T(5)*dt*vvv;
                vrep.simxSetObjectPosition(clientID,modelHandle,-1,r,vrep.simx_opmode_blocking);
            end
                        
            %{
            Original code --> spin to avoid obstacle
            leftLegJointValue=0;
            leftKneeJointValue=0;
            leftAnkleJointValue=0;
            leftShoulderJointValue=0;
            leftElbowJointValue=0;
            
            rightLegJointValue=0;
            rightKneeJointValue=0;
            rightAnkleJointValue=0;
            rightShoulderJointValue=0;
            rightElbowJointValue=0;
            
            if (movement==-1)
                [~,r]=vrep.simxGetObjectOrientation(clientID,modelHandle,-1,vrep.simx_opmode_blocking);
                r(3)=r(3)+dt*1.8*speedModulator;
                vrep.simxSetObjectOrientation(clientID,modelHandle,-1,r,vrep.simx_opmode_blocking);
            end
            if (movement==1)
                [~,r]=vrep.simxGetObjectOrientation(clientID,modelHandle,-1,vrep.simx_opmode_blocking);
                r(3)=r(3)-dt*1.8*speedModulator;
                vrep.simxSetObjectOrientation(clientID,modelHandle,-1,r,vrep.simx_opmode_blocking);
            end
            %}
            
        end
        
        vrep.simxSetJointPosition(clientID,legJointHandles(1),leftLegJointValue,vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,kneeJointHandles(1),leftKneeJointValue,vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,ankleJointHandles(1),leftAnkleJointValue,vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,shoulderJointHandles(1),leftShoulderJointValue,vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,elbowJointHandles(1),leftElbowJointValue,vrep.simx_opmode_blocking);
        
        vrep.simxSetJointPosition(clientID,legJointHandles(2),rightLegJointValue,vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,kneeJointHandles(2),rightKneeJointValue,vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,ankleJointHandles(2),rightAnkleJointValue,vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,shoulderJointHandles(2),rightShoulderJointValue,vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,elbowJointHandles(2),rightElbowJointValue,vrep.simx_opmode_blocking);
    end
    
    % Close file
    vrep.simxFinish(-1)
end

% Deconstruct object
vrep.delete();