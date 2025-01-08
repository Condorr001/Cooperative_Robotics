function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

if(mission.current_action == "safe_nav") 
    prev = mission.actions.safe_nav;
    current = mission.actions.safe_nav;
elseif(mission.current_action == "aligning") 
    prev = mission.actions.safe_nav;
    current = mission.actions.aligning;
elseif(mission.current_action == "landing") 
    prev = mission.actions.aligning;
    current = mission.actions.landing;
elseif(mission.current_action == "grasping") 
    prev = mission.actions.landing;
    current = mission.actions.grasping;
end

%% arm tool position control
% always active
uvms.A.t = eye(6);

%% vehicle position and orientation tasks
uvms.A.vp = eye(3) * ActionTransition("VP", prev, current, mission.phase_time);
uvms.A.vo = eye(3) * ActionTransition("VO", prev, current, mission.phase_time);

%% vehicle minimum altitude
delta_ma = 0.5;
min_alt = 1;
uvms.A.ma = DecreasingBellShapedFunction(min_alt, min_alt+delta_ma, 0, 1, uvms.altitude) .* ActionTransition("MA", prev, current, mission.phase_time);

%% zero vehicle altitude (landing)
uvms.A.lan = ActionTransition("LAN", prev, current, mission.phase_time);

%% horizontal attitude
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw; 
v_kv = [0 0 1]';
rho_ha = ReducedVersorLemma(v_kw,  v_kv);
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(rho_ha)) * ActionTransition("HA", prev, current, mission.phase_time);
%% vehicle alignment
uvms.A.hal = IncreasingBellShapedFunction(0.001, 0.05, 0, 1, norm(uvms.w_rho_hal)) * ActionTransition("HAL", prev, current, mission.phase_time);

%% zero velocity constraints
uvms.A.zvc = eye(6) * ActionTransition("ZVC", prev, current, mission.phase_time);

%% grasping
uvms.A.g = eye(6) * ActionTransition("G", prev, current, mission.phase_time);
end

