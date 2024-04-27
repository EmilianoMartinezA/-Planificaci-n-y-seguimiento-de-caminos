clear all
close all
clc

load('Path.mat')

figure()
map = binaryOccupancyMap(rot90(transpose(Path)), 10);
show(map)

ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];

planner_star = plannerRRTStar(ss,sv);
planner_star.ContinueAfterGoalReached = true;
planner_star.MaxIterations = 6000;
planner_star.MaxConnectionDistance = 5;

startLocation = [10 1 0];
endLocation = [8 9.5 0];

tic
[pthObj,solnInfo] = plan(planner_star,startLocation,endLocation);
time_rrt = toc;
text = ['tiempo ejecucion de RRT*: ',num2str(time_rrt)];
disp(text);

figure()
show(map)
hold on
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Conexión con Coppelia iniciada');
    
    [~, left_motor]=sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [~, right_motor]=sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    [~, Pioneer_p3dx]=sim.simxGetObjectHandle(clientID, ...
        'Pioneer_p3dx', sim.simx_opmode_blocking);

    l = 0.381 ; 
    Kp = 1;
    Kd = 0.1;
    kpr = 10;
    kpt = 10;
    dt = 0.01;
    theta = 0;
    Vmax = 1;
    wmax = pi;
    kact = 100;
    Thetae = [];	
    j = 1;
        
    while true
        if j > size(pthObj.States, 1)
            break;
        end
        posd = pthObj.States(j, 1:2);

        [~, Pioneer_Pos] = sim.simxGetObjectPosition(clientID, Pioneer_p3dx, ...
            -1, sim.simx_opmode_blocking);
        [~, Pioneer_Ori] = sim.simxGetObjectOrientation(clientID, Pioneer_p3dx, ...
            -1, sim.simx_opmode_blocking);

        theta_deseado = atan2((posd(2) - Pioneer_Pos(2)), (posd(1) - Pioneer_Pos(1))); 
        theta_error = Pioneer_Ori(3) - theta_deseado;

        if theta_error > pi
            theta_error = theta_error - 2*pi;
        elseif theta_error < -pi
            theta_error = theta_error + 2*pi;
        end

        w = -wmax*tanh(kpr*theta_error/wmax);
        d = sqrt((posd(1) - Pioneer_Pos(1))^2 + (posd(2) - Pioneer_Pos(2))^2);

        act = -((tanh(kact*(abs(theta_error)-pi/8)))-1)/2 ;
        V = Vmax.*tanh(kpt.*d/Vmax).*act;

        if d < 0.1
            V = 0;
            j = j + 1;
            if j > size(pthObj.States, 1)
                j = size(pthObj.States, 1);
                w = 0;
            end
        end

        vl = V - w*l/2;
        vr = V + w*l/1;

        V = (vl + vr)/2;
        w = (vr - vl)/l;

        xp = V.*cos(theta);
        yp = V.*sin(theta);
        thetap = w;

        x = Pioneer_Pos(1) + xp.*dt;  
        y = Pioneer_Pos(1) + yp.*dt;
        theta = theta + thetap.*dt;

        sim.simxSetJointTargetVelocity(clientID, left_motor, V - w*0.1, sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity(clientID, right_motor, V + w*0.1, sim.simx_opmode_blocking);

        pause(0.1);
    end

    disp('Conexión con Coppelia Terminada');
    sim.simxFinish(clientID);
    sim.delete(); 
end
