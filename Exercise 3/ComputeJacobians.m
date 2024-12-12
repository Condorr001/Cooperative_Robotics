function [pandaArm] = ComputeJacobians(pandaArm,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, -0.0305686, 1.53975, 0.753872] ) 
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

% Jacobian from base to end-effector
pandaArm.bJe = geometricJacobian(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');
% THE JACOBIAN bJe has dimension 6x9 (the matlab model include the joint
% of the gripper). YOU MUST RESIZE THE MATRIX IN ORDER TO CONTROL ONLY THE
% 7 JOINTS OF THE ROBOTIC ARM. 
pandaArm.bJe = pandaArm.bJe(:,1:7);

wSt = eye(6);
wSt(4:6, 1:3) = (skew(pandaArm.wTb(1:3,1:3) * pandaArm.bTe(1:3,1:3) * pandaArm.eTt(1:3,4)))';
pandaArm.wSt = wSt;
pandaArm.wJt = wSt * ([pandaArm.wTb(1:3,1:3) zeros(3); zeros(3) pandaArm.wTb(1:3,1:3)] * pandaArm.bJe); % 6x6 * (6x6 * 6x7)

if mission.phase == 2
    tTo_pos = pandaArm.wTo(1:3,4) - pandaArm.wTt(1:3,4); % distance between frames <o> and <t> 
    wSo = eye(6);
    wSo(4:6, 1:3) = -skew(tTo_pos);
    % wSo_L = [eye(3), zeros(3); -skew(tTo_L_pos), eye(3)]; 
    pandaArm.wJo = wSo * pandaArm.wJt;

    pandaArm.Jrc = pandaArm.wJo;
end

% joint limits 
pandaArm.Jjl = eye(7); %identity because we directly control the velocity of the joints

% minimum altitude
% For the minimum altitude jacobian, we take the z-coordinate of the tool, 
% so the last row of the jacobian wJt
pandaArm.Jma = pandaArm.wJt(6,:);

end
