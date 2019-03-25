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
    r1=[];
    r2=[];
    r3=[];
    
    if (clientID>-1)
        disp('Connected to remote API server');
        
        % get handle for Baxter_rightArm_joint1
        [res,handle_rigArmjoint1] = vrep.simxGetObjectHandle(clientID,'motor1',vrep.simx_opmode_blocking ); 
        [res,handle_rigArmjoint2] = vrep.simxGetObjectHandle(clientID,'motor2',vrep.simx_opmode_blocking ); 
        [res,handle_rigArmjoint3] = vrep.simxGetObjectHandle(clientID,'motor3',vrep.simx_opmode_blocking ); 

        
        while(vrep.simxGetConnectionId(clientID) ~= -1), % while v-rep connection is still active
            t = vrep.simxGetLastCmdTime(clientID) / 1000.0 % get current simulation time
            if (t > 30) break; 
            end % stop after t = 5 seconds
            
            [res,r1angle]=vrep.simxGetJointPosition(clientID,handle_rigArmjoint1,vrep.simx_opmode_blocking );
            [res,r2angle]=vrep.simxGetJointPosition(clientID,handle_rigArmjoint2,vrep.simx_opmode_blocking );
            [res,r3angle]=vrep.simxGetJointPosition(clientID,handle_rigArmjoint3,vrep.simx_opmode_blocking );

            r1= [r1 r1angle];
            r2= [r2 r2angle];
            r3= [r3 r3angle];

        end
        
        figure(1)
        plot(r1);hold on;plot(r2);hold on;plot(r3)
        % save data
        r=[r1' r2' r3'];
        file=fopen('angle.txt','wt');
        [m,n]=size(r);
        for i=1:1:m
            for j=1:1:n
                if j==n 
                    fprintf(file,'%g\n',r(i,j));
                else
                    fprintf(file,'%g\t',r(i,j));
                end
            end
        end
        fclose(file); 
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
