function [A] = ActionTransition(taskname, previous, current, time)
% This function produce the sigmoid function to activate or deactivate tasks
% according to the current and previous phase of the mission.
% Takes as input the current task name ("T","RC","JL","MA"), the previous
% task, the current one and the mission time. If doubts, see the Matlab doc of
% ismember()

% Added condition: given that the robots need to move sinchronoulsy
% immediately after grasping an object, the RC task should be activated
% (almost) immedialtely, not with a bell shaped function. Otherwise, the initial
% movement of the two arms would not be synchronized, which could cause
% faults and damages, other than an inclination of the object.
if (ismember(taskname, previous) == 0 && ismember(taskname, current) && taskname == "RC")
    A = 1;
elseif ((ismember(taskname, previous) && ismember(taskname, current) == 0) && taskname == "RC")
    A = 0;
elseif (ismember(taskname, previous) && ismember(taskname, current))
    A = 1;    
elseif (ismember(taskname, previous) == 0 && ismember(taskname, current))
    A = IncreasingBellShapedFunction(0, 1, 0, 1, time);
elseif (ismember(taskname, previous) && ismember(taskname, current) == 0)
    A = DecreasingBellShapedFunction(0, 1, 0, 1, time);
else
    A = 0;

end

