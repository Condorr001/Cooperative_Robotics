function [pandaArm] = UpdateTransforms(pandaArm,mission)
% the function updates all the transformations

% Left arm transformations
pandaArm.bTe = getTransform(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');

% <e> to <w>
pandaArm.wTe = pandaArm.wTb * pandaArm.bTe; % wTb is eye if left arm

% Transformation matrix from <t> to <w>
pandaArm.wTt = pandaArm.wTe * pandaArm.eTt;

if(mission.phase == 1)
    tRw = pandaArm.wTt(1:3,1:3)';    % rotation
    wrt = pandaArm.wTt(1:3,4);      % position
    pandaArm.tTo = [tRw -tRw'*wrt; 0 0 0 1] * pandaArm.wTo;
end

% <o> to <w> : ASSUME <t> = <g> during entire cooperation phase
if (mission.phase == 2)
    pandaArm.wTo = pandaArm.wTt * pandaArm.tTo; 
end

