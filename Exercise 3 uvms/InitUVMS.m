function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be computed at each time step
uvms.wTgv = eye(4,4);
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;

uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
uvms.Jvp = [];
uvms.Jvo = [];
uvms.Jma = [];
uvms.Jlan = [];

uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
uvms.xdot.vp = [];
uvms.xdot.vo = [];
uvms.xdot.ma = [];
uvms.xdot.lan = [];
    
uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.ha = zeros(1,1);
uvms.A.t = zeros(6,6);
uvms.A.vp = zeros(3);
uvms.A.vo = zeros(3);
uvms.A.v_lin = zeros(3);
uvms.A.ma = 0;
uvms.A.lan = 0;

uvms.theta = 0;
uvms.rock_center = zeros(3,1);
uvms.xdot.hal = [];
uvms.A.hal = zeros(1,1);
uvms.Jhal = [];
uvms.w_rho_hal = zeros(3,1);

uvms.xdot.zvc = [];
uvms.A.zvc = zeros(6,6);
uvms.Jzvc = [];

uvms.xdot.g = [];
uvms.A.g = zeros(6,6);

uvms.flag = 0;
uvms.landingflag = 0;
end

