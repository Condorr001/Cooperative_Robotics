function [ plt ] = UpdateDataPlot( plt, pandaArm, t, loop, mission )

% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script


plt.t(:, loop) = t;
plt.q(:, loop) = pandaArm.ArmL.q;
plt.q_dot(:, loop) = pandaArm.ArmL.q_dot;
plt.q2(:, loop) = pandaArm.ArmR.q;
plt.q_dot2(:, loop) = pandaArm.ArmR.q_dot;


%% mission 
plt.phase(loop) = mission.phase;

% Plot: desired object velocity
% left arm, cartesian
plt.armL.xdot_tool(:, loop) = pandaArm.ArmL.xdot.tool;
% right arm, cartesian
plt.armR.xdot_tool(:, loop) = pandaArm.ArmR.xdot.tool;

% End effector velocities
% left arm, cartesian 
plt.armL.xdot(:, loop) = pandaArm.ArmL.x;
% right arm, cartesian 
plt.armR.xdot(:, loop) = pandaArm.ArmR.x;

% Plot: manipulability task activation function

% Left arm
% activation rigid constarint
plt.a_rc(:, loop) = diag(pandaArm.A.rc);
% activation joint limits 
plt.armL.a_jl(:, loop) = diag(pandaArm.A.ArmL.jl);
% activation minimum altitude 
plt.armL.a_ma(:, loop) = pandaArm.A.ArmL.ma;
% activation tool 
plt.armL.a_tool(:, loop) = diag(pandaArm.A.tool(1:6,1:6));

% Right arm
% activation joint limits 
plt.armR.a_jl(:, loop) = diag(pandaArm.A.ArmR.jl);
% activation minimum altitude 
plt.armR.a_ma(:, loop) = pandaArm.A.ArmR.ma;
% activation tool 
plt.armR.a_tool(:, loop) = diag(pandaArm.A.tool(7:12,7:12));

plt.armR.dist_tools(:,loop) = pandaArm.ArmR.dist_tools;

end