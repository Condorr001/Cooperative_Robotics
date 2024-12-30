function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.toolPos(:, loop) = uvms.wTt(1:3,4);

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;

plt.A.ma(:,loop) = uvms.A.ma;
plt.A.ha(:,loop) = uvms.A.ha;
plt.A.vp(:,loop) = [uvms.A.vp(1,1),uvms.A.vp(2,2),uvms.A.vp(3,3)]';
plt.A.vo(:,loop) = [uvms.A.vo(1,1),uvms.A.vo(2,2),uvms.A.vo(3,3)]';
plt.A.zvc(:,loop) = [uvms.A.zvc(1,1), uvms.A.zvc(2,2),uvms.A.zvc(3,3), uvms.A.zvc(4,4), uvms.A.zvc(5,5), uvms.A.zvc(6,6)]';
plt.A.hal(:,loop) = uvms.A.hal;
plt.A.g(:,loop) = [uvms.A.g(1,1), uvms.A.g(2,2),uvms.A.g(3,3), uvms.A.g(4,4), uvms.A.g(5,5), uvms.A.g(6,6)]';
plt.A.lan(:,loop) = uvms.A.lan;


plt.xdot_des(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;

% plt.a(1:7, loop) = diag(uvms.A.jl);
% plt.a(8, loop) = uvms.A.mu;
% plt.a(9, loop) = uvms.A.ha(1,1);

% Vehicle Position and Orientation
plt.vpos(:,loop) = uvms.p(1:3);
plt.vorient(:,loop) = uvms.p(4:6);

% Real Vehicle linear and angular velocity
plt.vpos_dot(:,loop) = uvms.p_dot(1:3);
plt.vorient_dot(:,loop) = uvms.p_dot(4:6);

% Desired Vehicle linear and angular velocity
plt.vpos_dot_des(:,loop) = uvms.xdot.vp;
plt.vorient_dot_des(:,loop) = uvms.xdot.vo;

% plt.toolx(:,loop) = uvms.wTt(1,4);
% plt.tooly(:,loop) = uvms.wTt(2,4);
end