function [ plt ] = UpdateDataPlot( plt, pandaArm1, pandaArm2, t, loop, mission )


% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(:, loop) = t;
plt.q(:, loop) = pandaArm1.q;
plt.q_dot(:, loop) = pandaArm1.q_dot;
plt.q2(:, loop) = pandaArm2.q;
plt.q_dot2(:, loop) = pandaArm2.q_dot;


%% mission 
plt.phase(loop) = mission.phase;

% Plot: desired object velocity
% left arm, cartesian
plt.arm1.xdot_tool(:, loop) = pandaArm1.xdot.tool;
% right arm, cartesian
plt.arm2.xdot_tool(:, loop) = pandaArm2.xdot.tool;

% End effector velocities
% left arm, cartesian 
plt.arm1.xdot(:, loop) = pandaArm1.x;
% right arm, cartesian 
plt.arm2.xdot(:, loop) = pandaArm2.x;

% Plot: manipulability task activation function

% Left arm
% activation rigid constarint
plt.a_rc(:, loop) = diag(pandaArm1.A.rc);
% activation joint limits 
plt.arm1.a_jl(:, loop) = diag(pandaArm1.A.jl);
% activation minimum altitude 
plt.arm1.a_ma(:, loop) = pandaArm1.A.ma;
% activation tool 
plt.arm1.a_tool(:, loop) = diag(pandaArm1.A.tool(1:6,1:6));

% Right arm
% activation joint limits 
plt.arm2.a_jl(:, loop) = diag(pandaArm2.A.jl);
% activation minimum altitude 
plt.arm2.a_ma(:, loop) = pandaArm2.A.ma;
% activation tool 
plt.arm2.a_tool(:, loop) = diag(pandaArm2.A.tool(1:6,1:6));

end