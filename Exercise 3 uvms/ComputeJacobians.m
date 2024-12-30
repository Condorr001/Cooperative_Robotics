function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
%% DON'T CHANGE ----------------------------------------------------------
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];
%% UNTIL HERE ------------------------------------------------------------

% vehicle position Jacobian projected on <w>
uvms.Jvp = [zeros(3,7) -uvms.wTv(1:3,1:3) zeros(3)]; % 3x13, zeros for the arm as we're not controlling it here, 3 because we're only controlling the position of the vehicle (not orientation)
uvms.Jvo = [zeros(3,7) zeros(3) uvms.wTv(1:3,1:3)];
% uvms.Jvp = [zeros(3,7) -eye(3) zeros(3)];
% uvms.Jvo = [zeros(3,7), zeros(3) eye(3)];

% vehicle minimum altitude and lan Jacobians
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;
uvms.Jma = [zeros(1,7) v_kw' zeros(1,3)]; % ma is always 1x..., in this case 1x13 -> controlling only z-axis, for both vehicle and arm
uvms.Jlan = [zeros(1,7), v_kw', zeros(1,3)];

% horizontal attitude
v_kv = [0 0 1]';
rho_ha = ReducedVersorLemma(v_kw,  v_kv);
if (norm(rho_ha) > 0) 
    n_ha = rho_ha / norm(rho_ha);
    uvms.Jha = [zeros(1,7) zeros(1,3) n_ha'];
else
    uvms.Jha = [zeros(1,13)];
end

% Vehicle alignment to the target task
w_xv = uvms.wTv(1:3, 1:3) * [1 0 0]';
w_xv_perpendicular = (eye(3, 3) - w_kw * w_kw') * w_xv;
w_misalignment = ( eye(3, 3) - w_kw * w_kw' ) * ( uvms.rock_center - uvms.p(1:3) );
w_mis_norm = w_misalignment / norm( w_misalignment );
uvms.w_rho_hal = ReducedVersorLemma( w_mis_norm, w_xv_perpendicular);
w_v_align = ( skew(w_mis_norm)*w_xv_perpendicular ) / sin(norm(uvms.w_rho_hal));
uvms.Jhal = [ zeros(1, 7), zeros(1, 3), w_v_align'*uvms.wTv(1:3, 1:3) ];

% Zero Velocity Constraint
uvms.Jzvc = [ zeros(6, 7), [uvms.wTv(1:3, 1:3); zeros(3)], [zeros(3); uvms.wTv(1:3, 1:3)]];

% JL
uvms.Jjl = eye(7);
end