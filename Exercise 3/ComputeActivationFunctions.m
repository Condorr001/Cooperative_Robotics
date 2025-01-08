function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        % pandaArm.A.tool = ...;
        prev = mission.actions.go_to.tasks;
        current = prev;
    case 2 % Move the object holding it firmly
        % % Move-To

        prev = mission.actions.go_to.tasks;
        current = mission.actions.coop_manip.tasks;
        
    case 3 % STOP any motion 
        prev = mission.actions.coop_manip.tasks;
        current = mission.actions.end_motion.tasks;
end

% Move-To
pandaArm.A.tool = eye(6) * ActionTransition("T", prev, current, mission.phase_time);


% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
min_alt = 0.15;
delta_ma = 0.05;
pandaArm.A.ma = DecreasingBellShapedFunction(min_alt, min_alt+delta_ma, 0, 1, pandaArm.altitude) .* ActionTransition("MA", prev, current, mission.phase_time);

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error

n = size(pandaArm.q);
margin = 0.9; % max - max error, so 1 - 0.1

% for each joint in the arm
for i = 1:n(1)
    % check the sign of jlmin to know which are the lower and higher value
    % to put in the bell function, considering the margin
    if (pandaArm.jlmin(i) < 0)
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i)*margin, 0, 1, pandaArm.q(i));
    else
        minBell = DecreasingBellShapedFunction(pandaArm.jlmin(i)*margin, pandaArm.jlmin(i), 0, 1, pandaArm.q(i));
    end
    
    % same for jlmax, in this case with an increasing bell function
    if (pandaArm.jlmax(i) < 0)
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i), pandaArm.jlmax(i)*margin, 0, 1, pandaArm.q(i));
    else
        maxBell = IncreasingBellShapedFunction(pandaArm.jlmax(i)*margin, pandaArm.jlmax(i), 0, 1, pandaArm.q(i));
    end

    % sum the two bells to have a behaviour which respects both minimum and
    % maximum limits of each joint
    pandaArm.A.jl(i,i) = minBell + maxBell;
    
end
    
pandaArm.A.jl = pandaArm.A.jl .* ActionTransition("JL", prev, current, mission.phase_time);
end