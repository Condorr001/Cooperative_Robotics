function [pandaArm] = InitRobot(model,wTb_left,wTb_right)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArm containing the two robot
% model
pandaArm.ArmL = model;
pandaArm.ArmR = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArm.ArmL.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArm.ArmR.q = pandaArm.ArmL.q;
pandaArm.ArmL.q_dot = [0 0 0 0 0 0 0]';
pandaArm.ArmR.q_dot = [0 0 0 0 0 0 0]';
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka,[pandaArm.ArmL.q',0,0],'panda_link7');
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');
pandaArm.ArmL.wTb = wTb_left;
pandaArm.ArmR.wTb = wTb_right;
pandaArm.ArmL.wTe = pandaArm.ArmL.wTb*pandaArm.ArmL.bTe;
pandaArm.ArmR.wTe = pandaArm.ArmR.wTb*pandaArm.ArmR.bTe;

% joint limits corresponding to the actual Panda by Franka arm configuration
pandaArm.jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
pandaArm.jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% Init relevance Jacobians
pandaArm.ArmL.bJe = eye(6,7);
pandaArm.ArmR.bJe = eye(6,7);
pandaArm.Jjl = zeros(12, 14);
pandaArm.Jma = zeros(12, 14);
pandaArm.Jrc = zeros(12, 14);

%% ... TO HERE
% Init Task Reference vectors
pandaArm.xdot.rc = zeros(12, 1);

% Left arm
pandaArm.ArmL.xdot.jl = zeros(6, 1);
pandaArm.ArmL.xdot.tool = zeros(6, 1);

pandaArm.ArmL.wTt = zeros(4);
pandaArm.ArmL.wTg = zeros(4);
pandaArm.ArmL.wTo = zeros(4);
pandaArm.ArmL.tTo = zeros(4);

pandaArm.ArmL.wSt = zeros(6);
% pandaArm.ArmR.Jt_a = [];
% pandaArm.ArmR.Jt_v = [];
pandaArm.ArmL.wJt = zeros(6, 7);
% pandaArm.ArmR.Sto = [];
% pandaArm.ArmR.wJo = [];

pandaArm.ArmL.altitude = 0;

% Right arm
pandaArm.ArmR.xdot.jl = zeros(6, 1);
pandaArm.ArmR.xdot.tool = zeros(6, 1);

pandaArm.ArmR.wTt = zeros(4);
pandaArm.ArmR.wTg = zeros(4);
pandaArm.ArmR.wTo = zeros(4);
pandaArm.ArmR.tTo = zeros(4);

pandaArm.ArmR.wSt = zeros(6);
% pandaArm.ArmR.Jt_a = [];
% pandaArm.ArmR.Jt_v = [];
pandaArm.ArmR.wJt = zeros(6, 7);
% pandaArm.ArmR.Sto = [];
% pandaArm.ArmR.wJo = [];

pandaArm.ArmR.altitude = 0;

% Init Activation function for activate or deactivate tasks
% tool and rigid constraint consider angular and linear velocities
pandaArm.ArmL.A.tool = zeros(6);
pandaArm.ArmR.A.tool = zeros(6);
pandaArm.A.rc = zeros(12);
% jl is a matrix
pandaArm.ArmL.A.jl = zeros(7);
pandaArm.ArmR.A.jl = zeros(7);
pandaArm.A.ma = zeros(12);




end

