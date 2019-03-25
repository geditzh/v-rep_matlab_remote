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


function calibriation()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    k=0;
    
function [pos,quat] = getTargetPosVectorFromObjectPose(clientID,object,target)
    [~,pos] = vrep.simxGetObjectPosition(clientID,object,target,vrep.simx_opmode_blocking );
    [~,quat] = vrep.simxGetObjectQuaternion (clientID,object,target,vrep.simx_opmode_blocking );
end

    if (clientID>-1)
        disp('Connected to remote API server');        
        % get handle for Baxter_rightArm_joint1
        [~,target_3pss_s] = vrep.simxGetObjectHandle(clientID,'3pss_s_target',vrep.simx_opmode_blocking ); 
        [~,targetBase_3pss_s] = vrep.simxGetObjectHandle(clientID,'3PSS_S_base',vrep.simx_opmode_blocking ); 
        [~,fanuc_base] = vrep.simxGetObjectHandle(clientID,'fanuc_base',vrep.simx_opmode_blocking ); 
        [targetP,targetO]=getTargetPosVectorFromObjectPose(clientID,targetBase_3pss_s,fanuc_base); 
        
        while(vrep.simxGetConnectionId(clientID) ~= -1), % while v-rep connection is still active
            if (k==1) break;
            end  
            
            targetP.*1000
            targetO
            
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
