function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 22;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% rock position 
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
uAltitude.setup();

% Preallocation
plt = InitDataPlot(maxloops);

%% initialize uvms structure
uvms = InitUVMS('Robust');
%% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 
%% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [8.5 38.5 -36 0 -0.06 0.5]'; 
% % % % uvms.p = [48.5 11.5 -33   0 0 0]'; 

%% defines the goal position for the end-effector/tool position task
% uvms.goalPosition = [50   -12.5  -33]'; % too close to the sea floor
uvms.goalPosition = [10.5   37.5   -38]'; % far away from the sea floor
uvms.wRg = rotation(0, -0.06, 0.5);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

%% defines the goal position for the vehicle position task
% uvms.vehicleGoalPosition = [50   -12.5  -33]';
uvms.vehicleGoalPosition = [10.5   37.5   -38]'; % far away from the sea floor
uvms.wRgv = rotation(0, 0, 0);
uvms.wTgv = [uvms.wRgv uvms.vehicleGoalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

% Mission Configuration
mission.previous_action = "safe_nav";
mission.current_action = "safe_nav";
mission.phase = 1;
mission.phase_time = 0;

% Define the active tasks for each phase of the mission
%
% 1- MA  = Minimum Altitude Task
% 2- HA  = Horizontal Attitude Task
% 3- VP  = Vehicle Position Control Task
% 4- VO  = Vehicle Orientation Control Task
% 5- LAN = Zero Altitude Control Task (Landing Action)
% 6- HAL = Horizontal Alignment
% 7- G   = Grasping (Arm)
% 8- ZVC = Zero velocity Constraint (Vehicle)

mission.actions.safe_nav = ["MA", "HA", "VP", "VO"];
mission.actions.aligning = ["MA", "HAL", "VP", "VO"];
mission.actions.landing = ["HA", "LAN", "VP", "VO"];
mission.actions.grasping = ["ZVC", "G"];

uvms = ReceiveUdpPackets(uvms, uAltitude);
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;
uvms.altitude = v_kw' * [0 0 uvms.sensorDistance]';

uvms.rock_center = rock_center;

tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
    w_kw = [0 0 1]';
    v_kw = uvms.vTw(1:3,1:3) * w_kw;
    uvms.altitude = v_kw' * [0 0 uvms.sensorDistance]';

    % prova alignment with rock
    pos_misalignment = uvms.wTv(1:3,1:3)'*(rock_center - uvms.p(1:3));
    uvms.theta = atan2(pos_misalignment(2),pos_misalignment(1));
    
    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(13,1);
    Qp = eye(13); 
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    %[Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10);

    if (mission.phase == 1)
        disp('Phase 1');
        [Qp, ydotbar] = iCAT_task(uvms.A.ma,    uvms.Jma,    Qp,  ydotbar,  uvms.xdot.ma,   0.0001,   0.01, 10); % MA
        [Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp,  ydotbar,  uvms.xdot.ha,   0.0001,   0.01, 10); % HA
        [Qp, ydotbar] = iCAT_task(uvms.A.vp,    uvms.Jvp,    Qp,  ydotbar,  uvms.xdot.vp,   0.0001,   0.01, 10); % VP
        [Qp, ydotbar] = iCAT_task(uvms.A.vo,    uvms.Jvo,    Qp,  ydotbar,  uvms.xdot.vo,   0.0001,   0.01, 10); % VO

    elseif (mission.phase == 2)
        [Qp, ydotbar] = iCAT_task(uvms.A.ma,    uvms.Jma,    Qp,  ydotbar,  uvms.xdot.ma,   0.0001,   0.01, 10); % MA
        [Qp, ydotbar] = iCAT_task(uvms.A.hal,    uvms.Jhal,    Qp,  ydotbar,  uvms.xdot.hal,   0.0001,   0.01, 10); % HAL
    elseif (mission.phase == 3)
        [Qp, ydotbar] = iCAT_task(uvms.A.lan,     uvms.Jlan,     Qp,  ydotbar,  uvms.xdot.lan,    0.0001,   0.01, 10); % LAN
        [Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp,  ydotbar,  uvms.xdot.ha,   0.0001,   0.01, 10); % HA
        [Qp, ydotbar] = iCAT_task(uvms.A.vp,    uvms.Jvp,    Qp,  ydotbar,  uvms.xdot.vp,   0.0001,   0.01, 10); % VP
    elseif (mission.phase == 4)
        % [Qp, ydotbar] = iCAT_task(uvms.A.jl,    uvms.Jjl,    Qp,  ydotbar,  uvms.xdot.jl,   0.0001,   0.01, 10); % JL
        [Qp, ydotbar] = iCAT_task(uvms.A.zvc,    uvms.Jzvc,    Qp,  ydotbar,  uvms.xdot.zvc,   0.0001,   0.01, 10); % ZVC
        [Qp, ydotbar] = iCAT_task(uvms.A.g,    uvms.Jt,    Qp,  ydotbar,  uvms.xdot.g,   0.0001,   0.01, 10); % G
    end


    % [Qp, ydotbar] = iCAT_task(uvms.A.ma,    uvms.Jma,    Qp,  ydotbar,  uvms.xdot.ma,   0.0001,   0.01, 10); % MA
    % [Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp,  ydotbar,  uvms.xdot.ha,   0.0001,   0.01, 10); % HA
    % [Qp, ydotbar] = iCAT_task(uvms.A.lan,     uvms.Jlan,     Qp,  ydotbar,  uvms.xdot.lan,    0.0001,   0.01, 10); % LAN
    % [Qp, ydotbar] = iCAT_task(uvms.A.vp,    uvms.Jvp,    Qp,  ydotbar,  uvms.xdot.vp,   0.0001,   0.01, 10); % VP
    % [Qp, ydotbar] = iCAT_task(uvms.A.vo,    uvms.Jvo,    Qp,  ydotbar,  uvms.xdot.vo,   0.0001,   0.01, 10); % VO

    % [Qp, ydotbar] = iCAT_task(uvms.A.t,     uvms.Jt,     Qp,  ydotbar,  uvms.xdot.t,    0.0001,   0.01, 10); % T
    
    [Qp, ydotbar] = iCAT_task(eye(13),     eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integration
    uvms.q_dot = ydotbar(1:7);
    uvms.p_dot = ydotbar(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    mission.phase_time = mission.phase_time + deltat;

    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        % % % t
        %uvms.sensorDistance
        %uvms.xdot.gv'
        % % % uvms.A.ma
    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
    % if(uvms.flag)
    %     fclose(uVehicle);
    %     fclose(uArm);
    %     PrintPlot(plt);
    %     return;
    % end
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end