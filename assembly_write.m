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

function assembly_write()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    %read the joint angle data from 'angle.txt'
    jointValue=load('angle.txt'); %A matrix of 7 x 150.Each column vector recorded the changes of each joint Angle 
    [m n]=size(jointValue);
    k=0;
    
    if (clientID>-1)
        disp('Connected to remote API server');
        
        % get handle for Baxter_rightArm_joint1
        [res,handle_rigArmjoint1] = vrep.simxGetObjectHandle(clientID,'motor1',vrep.simx_opmode_blocking ); 
        [res,handle_rigArmjoint2] = vrep.simxGetObjectHandle(clientID,'motor2',vrep.simx_opmode_blocking ); 
        [res,handle_rigArmjoint3] = vrep.simxGetObjectHandle(clientID,'motor3',vrep.simx_opmode_blocking ); 
        
        while(vrep.simxGetConnectionId(clientID) ~= -1), % while v-rep connection is still active
            if (k==1) break;
            end
            for i=1:m
                vrep.simxPauseCommunication(clientID,1); 
                vrep.simxSetJointPosition (clientID,handle_rigArmjoint1,jointValue(i,1),vrep.simx_opmode_oneshot); 
                vrep.simxSetJointPosition (clientID,handle_rigArmjoint2,jointValue(i,2),vrep.simx_opmode_oneshot); 
                vrep.simxSetJointPosition (clientID,handle_rigArmjoint3,jointValue(i,3),vrep.simx_opmode_oneshot); 
                 vrep.simxPauseCommunication(clientID,0);
                pause(0.05);
            end
            k=k+1;
        end
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
