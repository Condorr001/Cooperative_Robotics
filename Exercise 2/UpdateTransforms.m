function [pandaArm] = UpdateTransforms(pandaArm, mission)
% the function updates all the transformations

% Left arm transformations
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka, ...
    [pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT

% Right arm transformations
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka, ...
    [pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

% <e> to <w>
pandaArm.ArmL.wTe = pandaArm.ArmL.bTe;
pandaArm.ArmR.wTe = pandaArm.ArmR.wTb * pandaArm.ArmR.bTe;

% Transformation matrix from <t> to <w>
pandaArm.ArmL.wTt = pandaArm.ArmL.wTe * pandaArm.ArmL.eTt;
pandaArm.ArmR.wTt = pandaArm.ArmR.wTe * pandaArm.ArmR.eTt;

% if(mission.phase == 1)
%     tRw_L = pandaArm.ArmL.wTt(1:3,1:3)';    % rotation
%     wrt_L = pandaArm.ArmL.wTt(1:3,4);      % position
%     pandaArm.ArmL.tTo = [tRw_L -tRw_L'*wrt_L; 0 0 0 1] * pandaArm.ArmL.wTo;
% 
%     tRw_R = pandaArm.ArmR.wTt(1:3,1:3)';
%     wrt_R = pandaArm.ArmR.wTt(1:3,4);    
%     pandaArm.ArmR.tTo = [tRw_R -tRw_R*wrt_R; 0 0 0 1] * pandaArm.ArmR.wTo;
% end

% <o> to <w> : ASSUME <t> = <g> during entire cooperation phase
if (mission.phase == 2)
    pandaArm.ArmL.wTo = pandaArm.ArmL.wTt * pandaArm.ArmL.tTo; 
    pandaArm.ArmR.wTo = pandaArm.ArmR.wTt * pandaArm.ArmR.tTo;
    
end
