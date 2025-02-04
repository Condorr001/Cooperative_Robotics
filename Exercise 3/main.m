addpath('./simulation_scripts');
clc;
clear;
close all
real_robot = false;
%% Initialization - DON'T CHANGE ANYTHING from HERE ... 
% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 15;
loop = 1;
maxloops = ceil(end_time/deltat);
mission.phase = 1;
mission.phase_time = 0;
model = load("panda.mat");

% UDP Connection with Franka Interface
if real_robot == true
    hudprLeft = dsp.UDPReceiver('LocalIPPort',1501,'MaximumMessageLength',255);
    hudprRight = dsp.UDPReceiver('LocalIPPort',1503,'MaximumMessageLength',255);
    hudpsLeft = dsp.UDPSender('RemoteIPPort',1500);
    hudpsLeft.RemoteIPAddress = '127.0.0.1';
    hudpsRight = dsp.UDPSender('RemoteIPPort',1502);
    hudpsRight.RemoteIPAddress = '127.0.0.1';
else
    hudps = dsp.UDPSender('RemoteIPPort',1500);
    hudps.RemoteIPAddress = '130.251.36.137';
end
%% TO HERE

% Init robot model
wTb_left = eye(4); %fixed transformation word -> base left
wTb_right = [rotation(0, 0, pi), [1.06, -0.01, 0]'; 0, 0, 0, 1]; %fixed transformation word -> base right
pandaArm1 = InitRobot(model,wTb_left);
pandaArm2 = InitRobot(model,wTb_right);
% pandaArm2.q = [2.6973,-0.2762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]'

% Preallocation
plt = InitDataPlot(maxloops);

% Init object frame
obj_length = 0.06;
w_obj_pos = [0.5 0 0.59]';

pandaArm1.wTo = eye(4);
pandaArm1.wTo(1:3, 4) = w_obj_pos;
pandaArm2.wTo = pandaArm1.wTo;

theta = deg2rad(-44.9949); % FIXED ANGLE BETWEEN EE AND TOOL 
tool_length = 0.2124; % FIXED DISTANCE BETWEEN EE AND TOOL

% Define trasnformation matrix from ee to tool.
eRt = rotation(0, 0, theta);
pandaArm1.eTt = eye(4);
pandaArm1.eTt(1:3, 1:3) = eRt;
pandaArm1.eTt(1:3, 4) = [0, 0, tool_length]';
pandaArm2.eTt = pandaArm1.eTt;

% Transformation matrix from <t> to <w>
pandaArm1.wTt = pandaArm1.wTe * pandaArm1.eTt;
pandaArm2.wTt = pandaArm2.wTe * pandaArm2.eTt;

%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points.
tRg = rotation(0, deg2rad(20), 0);
grasping1 = w_obj_pos - [obj_length/2 0 0]';
grasping2 = w_obj_pos + [obj_length/2 0 0]';

pandaArm1.wTg = [pandaArm1.wTt(1:3,1:3) * tRg, grasping1; 0 0 0 1];
pandaArm2.wTg = [pandaArm2.wTt(1:3,1:3) * tRg, grasping2; 0 0 0 1];

% Second goal move the object
w_g_pos = [0.6 0.4 0.48]';

pandaArm1.wTog = eye(4);
pandaArm1.wTog(1:3, 4) = w_g_pos;
pandaArm2.wTog = eye(4);
pandaArm2.wTog(1:3, 4) = w_g_pos;

%% Mission configuration
mission.prev_action = "go_to";
mission.current_action = "go_to";

mission.phase = 1;
mission.phase_time = 0;
% Define the active tasks for each phase of the mission
% T = move tool task
% JL = joint limits task
% MA = minimum altitude task
% RC = rigid constraint task
mission.actions.go_to.tasks = ["MA","JL","T"];
mission.actions.coop_manip.tasks = ["MA","JL","RC","T"];
mission.actions.end_motion.tasks = ["MA", "JL"];

%% CONTROL LOOP
for t = 0:deltat:end_time

    % Receive UDP packets - DO NOT EDIT
    if real_robot == true
        dataLeft = step(hudprLeft);
        dataRight = step(hudprRight);
        % wait for data (to decide)
         if t == 0
             while(isempty(dataLeft))
                 dataLeft = step(hudprLeft);
                 pause(deltat);
             end
             while(isempty(dataRight))
                 dataRight = step(hudprRight);
                 pause(deltat);
             end
         end
        qL = typecast(dataLeft, 'double');
        qR = typecast(dataRight, 'double');
        pandaArms.ArmL.q = qL;
        pandaArms.ArmR.q = qR;
    end

    % update all the involved variables
    [pandaArm1] = UpdateTransforms(pandaArm1,mission);
    [pandaArm2] = UpdateTransforms(pandaArm2,mission);
    [pandaArm1] = ComputeJacobians(pandaArm1,mission);
    [pandaArm2] = ComputeJacobians(pandaArm2,mission);
    [pandaArm1] = ComputeActivationFunctions(pandaArm1,mission);
    [pandaArm2] = ComputeActivationFunctions(pandaArm2,mission);
    [pandaArm1] = ComputeTaskReferences(pandaArm1,mission);
    [pandaArm2] = ComputeTaskReferences(pandaArm2,mission);

    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(7,1);
    Qp = eye(7);
    ydotbar2 = zeros(7,1);
    Qp2 = eye(7);

    % Used by the Move-To task
    tool_jacobian_L = zeros(6, 7);
    tool_jacobian_R = zeros(6, 7);
    if (mission.phase == 1)
        % In this phase the tool frame coincide with the center of the
        % gripper
        tool_jacobian_L = pandaArm1.wJt;
        tool_jacobian_R = pandaArm2.wJt;
    elseif(mission.phase == 2)
        % In this phase the tool frame coincide with the object frame
        tool_jacobian_L = pandaArm1.wJo;
        tool_jacobian_R = pandaArm2.wJo;
    end
    
    
    % ADD minimum distance from table
    pandaArm1.altitude = pandaArm1.wTt(3,4);
    pandaArm2.altitude = pandaArm2.wTt(3,4);

    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    
    % First Manipulator TPIK (left)
    % Task: Tool Move-To
    [Qp, ydotbar] = iCAT_task(pandaArm1.A.ma, pandaArm1.Jma, Qp, ydotbar, pandaArm1.xdot.alt, 0.0001, 0.01, 10);  % MA
    [Qp, ydotbar] = iCAT_task(pandaArm1.A.jl, pandaArm1.Jjl, Qp, ydotbar, pandaArm1.xdot.jl, 0.0001, 0.01, 10);  % JL
    [Qp, ydotbar] = iCAT_task(pandaArm1.A.tool, tool_jacobian_L, Qp, ydotbar, pandaArm1.xdot.tool, 0.0001, 0.01, 10); % tool position and orientation
    [Qp, ydotbar] = iCAT_task(eye(7),...
        eye(7),...
        Qp,...
        ydotbar, zeros(7,1),...
        0.0001,   0.01, 10);    % this task should be the last one

    % Second manipulator TPIK (right)
    % Task: Tool Move-To
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.ma, pandaArm2.Jma, Qp2, ydotbar2, pandaArm2.xdot.alt, 0.0001, 0.01, 10);  % MA
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.jl, pandaArm2.Jjl, Qp2, ydotbar2, pandaArm2.xdot.jl, 0.0001, 0.01, 10);  % JL
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.tool, tool_jacobian_R, Qp2, ydotbar2, pandaArm2.xdot.tool, 0.0001, 0.01, 10); % tool position and orientation
    [Qp2, ydotbar2] = iCAT_task(eye(7),...
        eye(7),...
        Qp2,...
        ydotbar2, zeros(7,1),...
        0.0001,   0.01, 10);    % this task should be the last one
    
    %% COOPERATION hierarchy
    % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
    % % % % % % non_coop_xdot_1 = pandaArm1.xdot.tool;
    % % % % % % non_coop_xdot_2 = pandaArm2.xdot.tool;
    % % % % % % 
    % % % % % % % Task: Left Arm Cooperation
    % % % % % % coop_xdot = (non_coop_xdot_1 + non_coop_xdot_2)/2;
    % % % % % % 
    % % % % % % % this task should be the last one
    % % % % % % [Qp, ydotbar] = iCAT_task(pandaArm1.A.tool, tool_jacobian_L, Qp, ydotbar, coop_xdot, 0.0001, 0.01, 10); % tool position and orientation
    % % % % % % [Qp, ydotbar] = iCAT_task(pandaArm1.A.rc, pandaArm1.Jrc, Qp, ydotbar, pandaArm1.xdot.rc, 0.0001, 0.01, 10);  % RC
    % % % % % % [Qp, ydotbar] = iCAT_task(pandaArm1.A.ma, pandaArm1.Jma, Qp, ydotbar, pandaArm1.xdot.alt, 0.0001, 0.01, 10);  % MA
    % % % % % % [Qp, ydotbar] = iCAT_task(pandaArm1.A.jl, pandaArm1.Jjl, Qp, ydotbar, pandaArm1.xdot.jl, 0.0001, 0.01, 10);  % JL
    % % % % % % [Qp, ydotbar] = iCAT_task(eye(7),...
    % % % % % %     eye(7),...
    % % % % % %     Qp, ydotbar,...
    % % % % % %     zeros(7,1),...
    % % % % % %     0.0001,   0.01, 10);    
    % % % % % % % Task: Right Arm Cooperation
    % % % % % % 
    % % % % % % % this task should be the last one
    % % % % % % [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.tool, tool_jacobian_R, Qp2, ydotbar2, -coop_xdot, 0.0001, 0.01, 10); % tool position and orientation
    % % % % % % [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.rc, -pandaArm2.Jrc, Qp2, ydotbar2, pandaArm2.xdot.rc, 0.0001, 0.01, 10);  % RC
    % % % % % % [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.ma, pandaArm2.Jma, Qp2, ydotbar2, pandaArm2.xdot.alt, 0.0001, 0.01, 10);  % MA
    % % % % % % [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.jl, pandaArm2.Jjl, Qp2, ydotbar2, pandaArm2.xdot.jl, 0.0001, 0.01, 10);  % JL
    % % % % % % [Qp2, ydotbar2] = iCAT_task(eye(7),...
    % % % % % %     eye(7),....
    % % % % % %     Qp2, ydotbar2,...
    % % % % % %     zeros(7,1),...
    % % % % % %     0.0001,   0.01, 10);    

    % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
    non_coop_xdot1 = tool_jacobian_L * ydotbar;       % xdot = J * ydotbar 
    non_coop_xdot2 = tool_jacobian_R * ydotbar2;


    if (mission.phase == 2)        
        Qp = eye(7);
        Qp2 = eye(7);
    
        % computing H1 and H2       
        H1 = pandaArm1.wJo*pinv(pandaArm1.wJo);      % Hi = J * Jinv
        H2 = pandaArm2.wJo*pinv(pandaArm2.wJo);    
        C = [H1 -H2];    
    
        % computing weights       
        mu_0 = 0.01;
        mu_a = mu_0 + norm(pandaArm1.xdot.tool - non_coop_xdot1);  % desired velocity - non cooperative velocity        
        mu_b = mu_0 + norm(pandaArm2.xdot.tool - non_coop_xdot2);
    
        % computing coop velocities      
        coop_xdot1 = (1 / (mu_a + mu_b)) * (mu_a * non_coop_xdot1 + mu_b * non_coop_xdot2);
        coop_xdot2 = coop_xdot1;
    
        % computing feasible cooperative velocities        
        feasible_coop_xdot = [H1 zeros(6); zeros(6) H2] * (eye(12) - pinv(C)*C) * [coop_xdot1; coop_xdot2];
    
        % plot variables
        pandaArm1.feasible_coop_xdot = feasible_coop_xdot;
        pandaArm2.feasible_coop_xdot = pandaArm1.feasible_coop_xdot;

        % Compute Jacobians for cooperative joint velocity computation
        coop_jacobian_L = pandaArm1.wJo; % Use Jacobian for object from Arm1
        coop_jacobian_R = pandaArm2.wJo; % Use Jacobian for object from Arm2
        
        % Compute feasible cooperative joint velocities for plots
        feasible_coop_velocity_q1 = pinv(coop_jacobian_L) * feasible_coop_xdot(1:6);
        feasible_coop_velocity_q2 = pinv(coop_jacobian_R) * feasible_coop_xdot(7:12);
        
        % Store results in the robot structures for later use
        pandaArm1.feasible_coop_velocity_qdot = feasible_coop_velocity_q1;
        pandaArm2.feasible_coop_velocity_qdot = feasible_coop_velocity_q2;


        % Task: Arm1 (Left Arm) Cooperation
        [Qp, ydotbar] = iCAT_task(pandaArm1.A.tool, tool_jacobian_L, Qp, ydotbar, feasible_coop_xdot(1:6), 0.0001, 0.01, 10);  % CV
        [Qp, ydotbar] = iCAT_task(pandaArm1.A.ma, pandaArm1.Jma, Qp, ydotbar, pandaArm1.xdot.alt, 0.0001, 0.01, 10);  % MA
        [Qp, ydotbar] = iCAT_task(pandaArm1.A.jl, pandaArm1.Jjl, Qp, ydotbar, pandaArm1.xdot.jl, 0.0001, 0.01, 10);   % JL
        [Qp, ydotbar] = iCAT_task(eye(7), eye(7), Qp, ydotbar, zeros(7,1), 0.0001,   0.01, 10);   % this task should be the last one
    
        % Task: Arm2 (Right Arm) Cooperation
        [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.tool, tool_jacobian_R, Qp2, ydotbar2, feasible_coop_xdot(7:12), 0.0001, 0.01, 10); % CV
        [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.ma, pandaArm2.Jma, Qp2, ydotbar2, pandaArm2.xdot.alt, 0.0001, 0.01, 10);  % MA
        [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.jl, pandaArm2.Jjl, Qp2, ydotbar2, pandaArm2.xdot.jl, 0.0001, 0.01, 10);   % JL
        [Qp2, ydotbar2] = iCAT_task(eye(7), eye(7), Qp2, ydotbar2, zeros(7,1), 0.0001,   0.01, 10);  % this task should be the last one  
    
    end 

    % plot variables
    pandaArm1.non_coop_xdot = non_coop_xdot1;
    pandaArm2.non_coop_xdot = non_coop_xdot2;

    
    % get the two variables for integration
    pandaArm1.q_dot = ydotbar(1:7);
    pandaArm2.q_dot = ydotbar2(1:7);

    pandaArm1.x = tool_jacobian_L * pandaArm1.q_dot;
    pandaArm2.x = tool_jacobian_R * pandaArm2.q_dot;

    % Integration
	pandaArm1.q = pandaArm1.q(1:7) + pandaArm1.q_dot*deltat;    
    pandaArm2.q = pandaArm2.q(1:7) + pandaArm2.q_dot*deltat;  

    %Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    if real_robot == false
        pandaArm1.q = pandaArm1.q(1:7) + pandaArm1.q_dot*deltat; 
        pandaArm2.q = pandaArm2.q(1:7) + pandaArm2.q_dot*deltat; 
    end
    %Send udp packets [q_dot1, ..., q_dot7]
    if real_robot == true
        step(hudpsLeft,[t;pandaArm1.q_dot]);
        step(hudpsRight,[t;pandaArm2.q_dot]);
    else 
        step(hudps,[pandaArm1.q',pandaArm2.q'])
    end

    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [pandaArm1,pandaArm2,mission] = UpdateMissionPhase(pandaArm1,pandaArm2,mission);

    % Compute distance between tools for plotting
    pandaArm1.dist_tools = norm(pandaArm1.wTt(1:3, 4) - pandaArm2.wTt(1:3, 4));
  
    % Update data for plots
    plt = UpdateDataPlot(plt,pandaArm1,pandaArm2,t,loop, mission);

    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        t 
        mission.phase
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    SlowdownToRealtime(deltat);
    
end
PrintPlot(plt);
save('simdata')
