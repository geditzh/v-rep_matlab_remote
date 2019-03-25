% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function assembly_read()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    %read the joint angle data of 3PSS   
    trajectory=load('trajectory.txt'); %A matrix of 7 x 150.Each column vector recorded the changes of each joint Angle 
    [m,~]=size(trajectory);
    
function [pos,quat] = getTargetPosVectorFromObjectPose(clientID,object,target)
    [~,pos] = vrep.simxGetObjectPosition(clientID,object,target,vrep.simx_opmode_blocking );
    [~,quat] = vrep.simxGetObjectQuaternion (clientID,object,target,vrep.simx_opmode_blocking );
end

    if (clientID>-1)
        disp('Connected to remote API server');
        
        % get handle for 3PSS_S
        [~,PSS_motor1] = vrep.simxGetObjectHandle(clientID,'motor1',vrep.simx_opmode_blocking ); 
        [~,PSS_motor2] = vrep.simxGetObjectHandle(clientID,'motor2',vrep.simx_opmode_blocking ); 
        [~,PSS_motor3] = vrep.simxGetObjectHandle(clientID,'motor3',vrep.simx_opmode_blocking );   
        [~,PSS_S_base] = vrep.simxGetObjectHandle(clientID,'3PSS_S_base',vrep.simx_opmode_blocking ); 
        [~,PSS_S_tip] = vrep.simxGetObjectHandle(clientID,'3PSS_S_tip',vrep.simx_opmode_blocking ); 
 
        
        % get handle for fanuc
        [~,irb360_ikTarget] = vrep.simxGetObjectHandle(clientID,'irb360_ikTarget',vrep.simx_opmode_blocking );
        
        % get handle for other thing
        [~,flag] = vrep.simxGetObjectHandle(clientID,'flag',vrep.simx_opmode_blocking );    %自定义通信标志位
        [~,Collision] = vrep.simxGetCollisionHandle (clientID,'Collision',vrep.simx_opmode_blocking );  %碰撞检测
         
        while(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
%             t = vrep.simxGetLastCmdTime(clientID) / 1000.0 % get current simulation time
%             if (t > 10) break; 
%             end % stop after t = 5 seconds

            %设置装配起始点欧拉角 b-b0
            alpha = 0/180*pi; beta = 0/180*pi; gamma = -0/180*pi;
            [~,b0] = PSS_inv(91, 354, 50, 50, 0/180*pi, 0/180*pi, 0/180*pi, 30/180*pi);
            [~,b] = PSS_inv(91, 354, 50, 50, gamma, beta, alpha, 30/180*pi);
            PSS_S_jointValue = (b-b0)/1000;

            vrep.simxSetJointPosition (clientID,PSS_motor1,PSS_S_jointValue(1),vrep.simx_opmode_blocking); 
            vrep.simxSetJointPosition (clientID,PSS_motor2,PSS_S_jointValue(2),vrep.simx_opmode_blocking); 
            vrep.simxSetJointPosition (clientID,PSS_motor3,PSS_S_jointValue(3),vrep.simx_opmode_blocking); 
            
            %启动标志点，让fanuc移动到位置点
            vrep.simxSetObjectPosition(clientID,flag,-1,[0 0 0],vrep.simx_opmode_blocking);    %设置dummy的第一个标志位，从v-rep里面读取标志位      
            
            %3PSS_S 回home点
            pause(0.3)
            vrep.simxSetJointPosition (clientID,PSS_motor1,0,vrep.simx_opmode_blocking); 
            vrep.simxSetJointPosition (clientID,PSS_motor2,0,vrep.simx_opmode_blocking); 
            vrep.simxSetJointPosition (clientID,PSS_motor3,0,vrep.simx_opmode_blocking); 
            pause(0.1)

            %让3PSS移动到装配点
            PSS_S_jointValue_discrete=[];
            for i=0:50
                vrep.simxPauseCommunication(clientID,1); 
                PSS_S_jointValue_discrete(1) = i/50*PSS_S_jointValue(1);
                PSS_S_jointValue_discrete(2) = i/50*PSS_S_jointValue(2);
                PSS_S_jointValue_discrete(3) = i/50*PSS_S_jointValue(3);
                vrep.simxSetJointPosition (clientID,PSS_motor1,PSS_S_jointValue_discrete(1),vrep.simx_opmode_oneshot); 
                vrep.simxSetJointPosition (clientID,PSS_motor2,PSS_S_jointValue_discrete(2),vrep.simx_opmode_oneshot); 
                vrep.simxSetJointPosition (clientID,PSS_motor3,PSS_S_jointValue_discrete(3),vrep.simx_opmode_oneshot); 
                vrep.simxPauseCommunication(clientID,0);
                pause(0.01);
            end

            [~,Rotate] = vrep.simxGetObjectOrientation(clientID,PSS_S_tip,PSS_S_base,vrep.simx_opmode_blocking);    %XYZ的欧拉角
            
           for i=1:m
                vrep.simxPauseCommunication(clientID,1);      
                vrep.simxSetJointPosition (clientID,PSS_motor1,(trajectory(i,1)-b0)/1000,vrep.simx_opmode_oneshot); 
                vrep.simxSetJointPosition (clientID,PSS_motor2,(trajectory(i,2)-b0)/1000,vrep.simx_opmode_oneshot); 
                vrep.simxSetJointPosition (clientID,PSS_motor3,(trajectory(i,3)-b0)/1000,vrep.simx_opmode_oneshot); 
                vrep.simxPauseCommunication(clientID,0);
                pause(0.01);
            end
            break;
        end
        
        %将XYZ欧拉角转化成ZYX欧拉角
            r = myfun(Rotate(1),Rotate(2),Rotate(3));
            alpha = atan2(r(3,2),r(3,3))/pi*180;
            beta = atan2(-r(3,1),sqrt(r(3,3)^2+r(3,2)^2))/pi*180;
            gamma = atan2(r(2,1),r(1,1))/pi*180;
            Rotate_fianl = [alpha,beta,gamma]       %ZYX的欧拉角
            
    else
        disp('Failed connecting to remote API server');
    end
    
    % Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'3PSS-S_FANUC',vrep.simx_opmode_oneshot);

    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);

    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
    
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
    
end
