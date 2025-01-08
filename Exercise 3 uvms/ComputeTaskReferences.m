function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
gain = 1; % unique gain for all tasks, expect minimum altitude and 
          % horizontal alignment

% tool
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = gain * [ang; lin];
% limit the requested velocities (safety)
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), gain);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), gain);

% vehicle orientation
[v_rho, v_d] = CartError(uvms.vTw*uvms.wTgv , eye(4));
uvms.xdot.vp = gain * (zeros(3,1) - v_d);
uvms.xdot.vp = Saturate(uvms.xdot.vp, 1);
uvms.xdot.vo = gain * v_rho; 

% minimum altitude
% higher gain, it is important that this task is executed as fast as
% possible
min_alt = 1;
uvms.xdot.ma = 5 * (min_alt - uvms.altitude);

% landing
uvms.xdot.lan = gain * (0 - uvms.altitude);

% horizontal attitude
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw; 
v_kv = [0 0 1]';
rho_ha = ReducedVersorLemma(v_kw,  v_kv);
uvms.xdot.ha = gain * (0.1 - norm(rho_ha));


% horizontal alignment
uvms.xdot.hal = 1.0 * (0.0 - norm(uvms.w_rho_hal));

% zero velocity constraint
uvms.xdot.zvc = gain * (zeros(6,1) - uvms.p_dot);

% grasping
matrix_rock = [eye(3) uvms.rock_center; 0 0 0 1];
[ang, lin] = CartError(matrix_rock, uvms.wTv*uvms.vTt);
uvms.xdot.g = gain * ([zeros(3,1); lin]);
uvms.xdot.g(1:3) = Saturate(uvms.xdot.g(1:3), 0.2);
uvms.xdot.g(4:6) = Saturate(uvms.xdot.g(4:6), 0.2);