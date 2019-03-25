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


function luoxuanxian()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    %read the joint angle data from 'angle.txt'
    circleValue=load('circle.txt'); %A matrix of 7 x 150.Each column vector recorded the changes of each joint Angle 
    [m,~]=size(circleValue);
    Rotate=[];
    Rotate_final=[];
    k=0;
    
function [pos,quat] = getTargetPosVectorFromObjectPose(clientID,object,target)
    [~,pos] = vrep.simxGetObjectPosition(clientID,object,target,vrep.simx_opmode_blocking );
    [~,quat] = vrep.simxGetObjectQuaternion (clientID,object,target,vrep.simx_opmode_blocking );
end

    if (clientID>-1)
        disp('Connected to remote API server');        
        % get handle for 3PSS_S
        [~,target_3pss_s] = vrep.simxGetObjectHandle(clientID,'3pss_s_target',vrep.simx_opmode_blocking ); 
        [~,targetBase_3pss_s] = vrep.simxGetObjectHandle(clientID,'3PSS_S_base',vrep.simx_opmode_blocking );
        [~,targetgraph_3pss_s] = vrep.simxGetObjectHandle(clientID,'3PSS_S_Graph',vrep.simx_opmode_blocking );
        [~,target3PSS_S_tip] = vrep.simxGetObjectHandle(clientID,'3PSS_tip',vrep.simx_opmode_blocking );
                
        % get handle for fanuc
        [~,irb360_ikTarget] = vrep.simxGetObjectHandle(clientID,'irb360_ikTarget',vrep.simx_opmode_blocking );
        
        % get relative position
        [targetP_origin_3PSS_S,targetO_origin_3PSS_S]=getTargetPosVectorFromObjectPose(clientID,target_3pss_s,targetBase_3pss_s); 
        [targetP_origin_fanuc,targetO_origin_fanuc]=getTargetPosVectorFromObjectPose(clientID,irb360_ikTarget,targetBase_3pss_s); 
        
        while(vrep.simxGetConnectionId(clientID) ~= -1), % while v-rep connection is still active
            if (k==1) break;
            end       
            for i=1:m
                [~,targetO] = vrep.simxGetObjectQuaternion (clientID,targetgraph_3pss_s,targetBase_3pss_s,vrep.simx_opmode_blocking);
                vrep.simxPauseCommunication(clientID,1);     
                vrep.simxSetObjectPosition(clientID,target_3pss_s,targetBase_3pss_s,circleValue(i,:),vrep.simx_opmode_oneshot);  
                
                vrep.simxSetObjectPosition(clientID,irb360_ikTarget,targetBase_3pss_s,[circleValue(i,[1,2]) circleValue(i,3)+0.12],vrep.simx_opmode_oneshot);
                vrep.simxSetObjectQuaternion(clientID,irb360_ikTarget,targetBase_3pss_s,targetO,vrep.simx_opmode_oneshot);
                vrep.simxPauseCommunication(clientID,0);
                
                [~,Rotate(i,:)] = vrep.simxGetObjectOrientation(clientID,target3PSS_S_tip,targetBase_3pss_s,vrep.simx_opmode_blocking);              
                pause(0.008);
            end
           k=k+1;
        end
        Rotate
        
        for i=1:size(Rotate)
            r = myfun(Rotate(i,1),Rotate(i,2),Rotate(i,3));
            alpha = atan2(r(3,2),r(3,3))/pi*180;
            beta = atan2(-r(3,1),sqrt(r(3,3)^2+r(3,2)^2))/pi*180;
            gamma = atan2(r(2,1),r(1,1))/pi*180;
            Rotate_fianl(i,:) = [alpha,beta,gamma];
        end
        Rotate_fianl
        file=fopen('luoxuanxian.txt','wt');
        [m,n]=size(Rotate_fianl);
        for i=1:1:m
            for j=1:1:n
                if j==n 
                    fprintf(file,'%g\n',Rotate_fianl(i,j));
                else
                    fprintf(file,'%g\t',Rotate_fianl(i,j));
                end
            end
        end
        fclose(file); 
        
        vrep.simxSetObjectPosition(clientID,target_3pss_s,targetBase_3pss_s,targetP_origin_3PSS_S,vrep.simx_opmode_oneshot);
        vrep.simxSetObjectQuaternion(clientID,target_3pss_s,targetBase_3pss_s,targetO_origin_3PSS_S,vrep.simx_opmode_oneshot);      
        vrep.simxSetObjectPosition(clientID,irb360_ikTarget,targetBase_3pss_s,targetP_origin_fanuc,vrep.simx_opmode_oneshot);
        vrep.simxSetObjectQuaternion(clientID,irb360_ikTarget,targetBase_3pss_s,targetO_origin_fanuc,vrep.simx_opmode_oneshot);
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
