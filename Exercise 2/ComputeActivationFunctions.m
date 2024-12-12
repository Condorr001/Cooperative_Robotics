function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

% if(mission.current_action == "go_to")
%     previous_action_list = mission.actions.go_to.tasks;
%     current_action_list = mission.actions.go_to.tasks;
%  elseif(mission.current_action == "coop_manip")
%      previous_action_list = mission.actions.go_to.tasks;
%      current_action_list = mission.actions.coop_manip.tasks;
%  elseif(mission.current_action == "end_motion")
%      previous_action_list = mission.actions.coop_manip.tasks;
%      current_action_list = mission.actions.end_motion.tasks;
% end

%% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
         % pandaArm.A.tool = eye(12) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);
        prev = mission.actions.go_to.tasks;
        current = prev;
        task_rc = 0;

    case 2 % Move the object holding it firmly
        % % Rigid Grasp Constraint
        % pandaArm.A.rc = eye(6) .* ActionTransition("RC", previous_action_list, current_action_list, mission.phase_time);
        % % Move-To
        % pandaArm.A.tool = eye(12) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);

        prev = mission.actions.go_to.tasks;
        current = mission.actions.coop_manip.tasks;
        task_rc = 1;

    case 3 % STOP any motion 
        % pandaArm.A.rc = eye(6) .* ActionTransition("RC", previous_action_list, current_action_list, mission.phase_time);
        % 
        % pandaArm.A.tool = eye(12) .* ActionTransition("T", previous_action_list, current_action_list, mission.phase_time);

        prev = mission.actions.coop_manip.tasks;
        current = mission.actions.end_motion.tasks;
        task_rc = 1;      
end

% Move-To
pandaArm.A.tool = eye(12) * ActionTransition("T", prev, current, mission.phase_time);

% Rigid Grasp Constraint
if task_rc == 1
    pandaArm.A.rc = eye(6) * ActionTransition("RC", prev, current, mission.phase_time);
end

%% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
min_alt = 0.15;
delta_ma = 0.05;
pandaArm.A.ArmL.ma = DecreasingBellShapedFunction(min_alt, min_alt+delta_ma, 0, 1, pandaArm.ArmL.altitude) * ActionTransition("MA", prev, current, mission.phase_time);
pandaArm.A.ArmR.ma = DecreasingBellShapedFunction(min_alt, min_alt+delta_ma, 0, 1, pandaArm.ArmR.altitude) * ActionTransition("MA", prev, current, mission.phase_time);

pandaArm.A.ma = [pandaArm.A.ArmL.ma 0; 0 pandaArm.A.ArmR.ma];

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error

n = size(pandaArm.ArmL.q);
m = size(pandaArm.ArmR.q);
margin = 0.9; % max - max error, so 1 - 0.1

% for each joint in the left arm
for i = 1:n(1)
    % check the sign of jlmin to know which are the lower and higher value
    % to put in the bell function, considering the margin
    if (pandaArm.jlmin(i) < 0)
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i)*margin, 0, 1, pandaArm.ArmL.q(i));
    else
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i)*margin, pandaArm.jlmin(i), 0, 1, pandaArm.ArmL.q(i));
    end
    
    % same for jlmax, in this case with an increasing bell function
    if (pandaArm.jlmax(i) < 0)
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i), pandaArm.jlmax(i)*margin, 0, 1, pandaArm.ArmL.q(i));
    else
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i)*margin, pandaArm.jlmax(i), 0, 1, pandaArm.ArmL.q(i));
    end

    % sum the two bells to have a behaviour which respects both minimum and
    % maximum limits of each joint
    pandaArm.A.ArmL.jl(i,i) = minBell + maxBell;
    
end

% exact same for the right arm
for i = 1:m(1)
    if (pandaArm.jlmin(i) < 0)
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i)*margin, 0, 1, pandaArm.ArmR.q(i));
    else
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i)*margin, pandaArm.jlmin(i), 0, 1, pandaArm.ArmR.q(i));
    end
    if (pandaArm.jlmax(i) < 0)
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i), pandaArm.jlmax(i)*margin, 0, 1, pandaArm.ArmR.q(i));
    else
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i)*margin, pandaArm.jlmax(i), 0, 1, pandaArm.ArmR.q(i));
    end

    pandaArm.A.ArmR.jl(i,i) = minBell + maxBell;
    
end

pandaArm.A.jl = [pandaArm.A.ArmL.jl zeros(7,7); zeros(7,7) pandaArm.A.ArmR.jl] .* ActionTransition("JL", prev, current, mission.phase_time);

end