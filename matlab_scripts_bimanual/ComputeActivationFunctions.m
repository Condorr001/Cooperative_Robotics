function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

%% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        % pandaArm.A.tool = eye(6) * ActionTransition("T", mission.actions.go_to.tasks, mission.actions.go_to.tasks, mission.phase_time);

        prev = mission.actions.go_to.tasks;
        current = prev;
        task_rc = 0;

    case 2 % Move the object holding it firmly
        % % Rigid Grasp Constraint
        % pandaArm.A.rc = eye(6) * ActionTransition("RC", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time);
        % % Move-To
        % pandaArm.A.tool = eye(6) * ActionTransition("T", mission.actions.go_to.tasks, mission.actions.coop_manip.tasks, mission.phase_time);

        prev = mission.actions.go_to.tasks;
        current = mission.actions.coop_manip.tasks;
        task_rc = 1;

    case 3 % STOP any motion 
        % % Rigid Grasp Constraint
        % pandaArm.A.rc = eye(6) * ActionTransition("RC", mission.actions.coop_manip.tasks, mission.actions.end_motion.tasks, mission.phase_time);
        % % Move-To
        % pandaArm.A.tool = eye(6) * ActionTransition("T", mission.actions.coop_manip.tasks, mission.actions.end_motion.tasks, mission.phase_time);
        
        prev = mission.actions.coop_manip.tasks;
        current = mission.actions.end_motion.tasks;
        task_rc = 1;
        
end
% Move-To
pandaArm.A.tool = eye(6) * ActionTransition("T", prev, current, mission.phase_time);
% Rigid Grasp Constraint
if task_rc == 1
    pandaArm.A.rc = eye(6) * ActionTransition("RC", prev, current, mission.phase_time);
end

%% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
min_alt = 0.15;
delta_ma = 0.05;
pandaArm.A.ArmL.ma = DecreasingBellShapedFunction(min_alt, min_alt+delta_ma, 0, 1, pandaArm.ArmL.altitude) * ActionTransition("MA", prev, current, mission.phase_time);

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
delta_jl = (pandaArm.jlmax - pandaArm.jlmin)/10; % 10% delta
jl_activation_values_L = zeros(1, 7);
jl_activation_values_R = zeros(1, 7);

for i=1:7
    % Left arm
    jl_activation_values_L(i) = IncreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i)+delta_jl(i), 0, 1, pandaArm.ArmL.q(i)) + DecreasingBellShapedFunction(pandaArm.jlmax(i)-delta_jl(i), pandaArm.jlmax(i), 0, 1, pandaArm.ArmL.q(i));
    % Right arm
    jl_activation_values_R(i) = IncreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i)+delta_jl(i), 0, 1, pandaArm.ArmR.q(i)) + DecreasingBellShapedFunction(pandaArm.jlmax(i)-delta_jl(i), pandaArm.jlmax(i), 0, 1, pandaArm.ArmR.q(i));
end

pandaArm.A.ArmL.jl = diag(jl_activation_values_L) * ActionTransition("JL", prev, current, mission.phase_time);
pandaArm.A.ArmR.jl = diag(jl_activation_values_R) * ActionTransition("JL", prev, current, mission.phase_time);
pandaArm.A.jl = [pandaArm.A.ArmL.jl zeros(7); zeros(7) pandaArm.A.ArmR.jl];

end
