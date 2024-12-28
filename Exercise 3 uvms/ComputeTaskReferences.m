function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
gain = 0.2;
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = gain * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), gain);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), gain);

% w_vehicle_target_distance = uvms.wTgv(1:3,4) - uvms.wTv(1:3,4);
% uvms.xdot.vp = -0.6 * w_vehicle_target_distance;
% uvms.xdot.vp = Saturate(uvms.xdot.vp, 0.8);

% vehicle orientation
[v_rho, v_d] = CartError(uvms.vTw*uvms.wTgv , eye(4));
uvms.xdot.vp = gain * (zeros(3,1) - v_d);
uvms.xdot.vp = Saturate(uvms.xdot.vp, 1);
uvms.xdot.vo = gain * v_rho; 

% minimum altitude
min_alt = 1;
uvms.xdot.ma = 5 * (min_alt - uvms.altitude);

% landing
uvms.xdot.lan = 1 * (0 - uvms.altitude);

% horizontal attitude
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw; 
v_kv = [0 0 1]';
rho_ha = ReducedVersorLemma(v_kw,  v_kv);
uvms.xdot.ha = gain * (0.1 - norm(rho_ha));