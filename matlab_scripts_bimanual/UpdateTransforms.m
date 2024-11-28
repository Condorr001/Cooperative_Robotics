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

% <o> to <w> : ASSUME <t> = <g> during entire cooperation phase
if (mission.phase == 2)
    % if size(pandaArm.ArmL.wTo) == 0
    %     wTb_right = [cos(pi), -sin(pi), 0, 1.06;
    %          sin(pi), cos(pi), 0, -0.01;
    %          0,             0, 1,     0;
    %          0,             0, 0,     1]; % fixed transformation word -> base2
    % 
    %     pandaArms.ArmL.wTo = eye(4);
    %     pandaArms.ArmL.wTo(1:3, 4) = w_obj_pos;
    %     pandaArms.ArmR.wTo = wTb_right' * pandaArms.ArmL.wTo;
    %     pandaArm.ArmL.tTo = inv(pandaArm.ArmL.wTt) * pandaArm.ArmL.wTo;
    %     pandaArm.ArmR.tTo = inv(pandaArm.ArmR.wTt) * pandaArm.ArmR.wTo;
    % end

    pandaArm.ArmL.wTo = pandaArm.ArmL.wTt * pandaArm.ArmL.tTo; 
    pandaArm.ArmR.wTo = pandaArm.ArmR.wTt * pandaArm.ArmR.tTo;
    
end
