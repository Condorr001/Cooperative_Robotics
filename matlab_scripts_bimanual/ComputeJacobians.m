function [pandaArm] = ComputeJacobians(pandaArm,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, 
% -0.0305686, 1.53975, 0.753872] ) 
%
% remember: the control vector is:
% [q_dot] 
% [qdot_1, qdot_2, ..., qdot_7]
%
% therefore all task jacobians should be of dimensions
% m x 14
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]

% Left Arm base to ee Jacobian
pandaArm.ArmL.bJe = geometricJacobian(pandaArm.ArmL.franka, ...
    [pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
%consider only the columns we want
pandaArm.ArmL.bJe = pandaArm.ArmL.bJe(:,1:7);
% Right Arm base to ee Jacobian
pandaArm.ArmR.bJe = geometricJacobian(pandaArm.ArmR.franka, ...
    [pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT
pandaArm.ArmR.bJe = pandaArm.ArmR.bJe (:,1:7);

% Top three rows are angular velocities, bottom three linear velocities
wSt_L = eye(6);
wSt_L(4:6, 1:3) = (skew(pandaArm.ArmL.wTe(1:3,1:3) * pandaArm.ArmL.eTt(1:3,4)))';
% wSt_L = [eye(3), zeros(3); (skew(pandaArm.ArmL.wTe(1:3,1:3) * pandaArm.ArmL.eTt(1:3,4)))', eye(3)];
pandaArm.ArmL.wSt = wSt_L;
pandaArm.ArmL.wJt = wSt_L * pandaArm.ArmL.bJe; % 6x6 * 6x7 = 6x7

% for the right arm, we also need to consider the transformation between
% the world frame, which is coincident with the left arm base frame, and
% its base frame
wSt_R = eye(6);
wSt_R(4:6, 1:3) = (skew(pandaArm.ArmR.wTb(1:3,1:3) * pandaArm.ArmR.bTe(1:3,1:3) * pandaArm.ArmR.eTt(1:3,4)))';
% wSt_R = [eye(3), zeros(3); (skew(pandaArm.ArmR.wTb(1:3,1:3) * pandaArm.ArmR.bTe(1:3,1:3) * pandaArm.ArmR.eTt(1:3,4)))', eye(3)];
pandaArm.ArmR.wSt = wSt_R;
pandaArm.ArmR.wJt = wSt_R * ([pandaArm.ArmR.wTb(1:3,1:3) zeros(3); zeros(3) pandaArm.ArmR.wTb(1:3,1:3)] * pandaArm.ArmR.bJe); % 6x6 * (6x6 * 6x7)

if (mission.phase == 2) % so if the robot is grasping an object
    tTo_L_pos = pandaArm.ArmL.wTo(1:3,4) - pandaArm.ArmL.wTt(1:3,4); % distance between frames <o> and <t> 
    wSo_L = eye(6);
    wSo_L(4:6, 1:3) = -skew(tTo_L_pos);
    % wSo_L = [eye(3), zeros(3); -skew(tTo_L_pos), eye(3)]; 
    pandaArm.ArmL.wJo = wSo_L * pandaArm.ArmL.wJt; 
 
    tTo_R_pos = pandaArm.ArmR.wTo(1:3,4) - pandaArm.ArmR.wTt(1:3,4); % distance between frames <o> and <t>
    wSo_R = eye(6);
    wSo_R(4:6, 1:3) = -skew(tTo_R_pos);
    % wSo_R = [eye(3), zeros(3); -skew(tTo_R_pos), eye(3)]; 
    pandaArm.ArmR.wJo = wSo_R * pandaArm.ArmR.wJt; 

    pandaArm.Jrc = [pandaArm.ArmL.wJo -pandaArm.ArmR.wJo]; 
end

% Common Jacobians
% joint limits 
pandaArm.Jjl = eye(14); %identity because we directly control the velocity of the joints
% minimum altitude 
% For the minimum altitude jacobian, we take the z-coordinate of the tool, 
% so the last row of the jacobian wJt
% pandaArm.Jma = [pandaArm.ArmL.wJt(6,:) zeros(1,7); zeros(1,7) pandaArm.ArmR.wJt(6,:)]; % m x 14 -> 12 x 14
pandaArm.Jma = [pandaArm.ArmL.wJt, zeros(6,7); pandaArm.ArmR.wJt, zeros(6,7)];

end