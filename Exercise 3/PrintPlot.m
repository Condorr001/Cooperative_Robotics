function [ ] = PrintPlot( plt)



% LEFT ARM
fig = figure('Name', 'Joint position and velocity Left Arm');

% Subplot 1: Joint positions
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
grid on;
title('LEFT ARM');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
xlabel('Time [s]'); % X-axis label
ylabel('Joint Position [rad]'); % Y-axis label
% Subplot 2: Joint velocities
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
grid on;
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
xlabel('Time [s]'); % X-axis label
ylabel('Joint Velocity [rad/s]'); % Y-axis label

saveas(fig, 'LeftArm_JointPositionVelocity.fig');
print(fig, 'LeftArm_JointPositionVelocity', '-depsc');

% RIGHT ARM
fig = figure('Name', 'Joint position and velocity Right Arm');

% Subplot 1: Joint positions
subplot(2,1,1);
hplot = plot(plt.t, plt.q2);
grid on;
title('RIGHT ARM');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
xlabel('Time [s]'); % X-axis label
ylabel('Joint Position [rad]'); % Y-axis label
% Subplot 2: Joint velocities
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot2);
grid on;
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
xlabel('Time [s]'); % X-axis label
ylabel('Joint Velocity [rad/s]'); % Y-axis label

saveas(fig, 'RightArm_JointPositionVelocity.fig');
print(fig, 'RightArm_JointPositionVelocity', '-depsc');


%% Activation functions
% Left arm
fig = figure("Name","Left Arm Activation Function");
plot(plt.t(1:end), plt.arm1.a_jl(1,:), plt.t(1:end), plt.arm1.a_ma, plt.t(1:end), plt.arm1.a_tool(1,:), "LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Activation");
%fontsize(gca,22,"points");
title("Left Arm Activation Function");
%lgd = legend("MA","T","RC","Location","northeastoutside");
lgd = legend("JL", "MA","T");
%fontsize(lgd,22,"points");
ylim([-0.1 1.1]);
saveas(fig, 'LeftArm_ActivationFunction.fig');
print(fig, 'LeftArm_ActivateFunction', '-depsc');

% Right arm
fig = figure("Name","Right Arm Activation Function");
plot(plt.t(1:end), plt.arm2.a_jl(1,:), plt.t(1:end), plt.arm2.a_ma, plt.t(1:end), plt.arm2.a_tool(1,:), "LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Activation");
%fontsize(gca,22,"points");
title("Right Arm Activation Function");
%lgd = legend("MA","T","RC","Location","northeastoutside");
lgd = legend( "JL", "MA","T");
%fontsize(lgd,22,"points");
ylim([-0.1 1.1]);
saveas(fig, 'RightArm_ActivationFunction.fig');
print(fig, 'RightArm_ActivateFunction', '-depsc');

%% Desired vs end-effector angular velocities
% Left arm
fig = figure("Name", "Left Arm End-Effector Desired vs Actual Angular Velocities");
plot(plt.t(1:end), plt.arm1.xdot_tool(1:3, :),"LineWidth", 2);
hold on;
plot(plt.t(1:end), plt.arm1.xdot(1:3, :) ,"LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Velocities [rad/s]");
%fontsize(gca,22,"points");
title("Left Arm End-Effector Desired vs Actual Angular Velocities");
lgd = legend("w_x_{des}","w_y_{des}","w_z_{des}","w_x_{ee}","w_y_{ee}","w_z_{ee}");
%fontsize(lgd,22,"points");
saveas(fig, 'LeftArm_EEdesVSAngualar.fig');
print(fig, 'LeftArm_EEdesVSAngualar', '-depsc');

% Right arm
fig = figure("Name", "Right Arm End-Effector Desired vs Actual Angular Velocities");
plot(plt.t(1:end), plt.arm2.xdot_tool(1:3, :),"LineWidth", 2);
hold on;
plot(plt.t(1:end), plt.arm2.xdot(1:3, :) ,"LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Velocities [rad/s]");
%fontsize(gca,22,"points");
title("Right Arm End-Effector Desired vs Actual Angular Velocities");
lgd = legend("w_x_{des}","w_y_{des}","w_z_{des}","w_x_{ee}","w_y_{ee}","w_z_{ee}");
%fontsize(lgd,22,"points");
saveas(fig, 'RightArm_EEdesVSAngular.fig');
print(fig, 'RightArm_EEdesVSAngular', '-depsc');

%% Desired vs end-effector linear velocities
% Left arm
fig = figure("Name", "Left Arm End-Effector Desired vs Actual Linear Velocities");
plot(plt.t(1:end), plt.arm1.xdot_tool(4:6, :),"LineWidth", 2);
hold on;
plot(plt.t(1:end), plt.arm1.xdot(4:6, :) ,"LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Velocities [m/s]");
%fontsize(gca,22,"points");
title("Left Arm End-Effector Desired vs Actual Linear Velocities");
lgd = legend("v_x_{des}","v_y_{des}","v_z_{des}","v_x_{ee}","v_y_{ee}","v_z_{ee}");
%fontsize(lgd,22,"points");
saveas(fig, 'LeftArm_EEdesVSLinear.fig');
print(fig, 'LeftArm_EEdesVSLinear', '-depsc');

% Right arm
fig = figure("Name", "Right Arm End-Effector Desired vs Actual Linear Velocities");
plot(plt.t(1:end), plt.arm2.xdot_tool(4:6, :),"LineWidth", 2);
hold on;
plot(plt.t(1:end), plt.arm2.xdot(4:6, :) ,"LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Velocities [m/s]");
%fontsize(gca,22,"points");
title("Right Arm End-Effector Desired vs Actual Linear Velocities");
lgd = legend("v_x_{des}","v_y_{des}","v_z_{des}","v_x_{ee}","v_y_{ee}","v_z_{ee}");
%fontsize(lgd,22,"points");
saveas(fig, 'RightArm_EEdesVSLinear.fig');
print(fig, 'RightArm_EEdesVSLinear', '-depsc');


%% des left vel



%% des right vel

%% desired object linear velocity vs non cooperative cartesian velocities
% fig = figure("Name", "Desired object linear velocity vs non cooperative cartesian velocities");
% plot(plt.t(1:end), plt.arm1.non_coop_xdot(1:3, :),"LineWidth", 2);
% hold on;
% plot(plt.t(1:end), plt.arm2.non_coop_xdot(1:3, :) ,"LineWidth", 2);
% hold on;
% plot(plt.t(1:end), plt.feasible_coop_xdot(1:3, :) ,"LineWidth", 2);
% hold on;
% plot(plt.t(1:end), plt.feasible_coop_xdot(7:9, :) ,"LineWidth", 2);
% grid on;
% xlabel("Time [s]");
% ylabel("Velocities [m/s]");
% %fontsize(gca,22,"points");
% title("Desired object linear velocity vs non cooperative cartesian velocities");
% lgd = legend("v1_x_{noncoop}","v1_y_{noncoop}","v1_z_{noncoop}",...
%              "v2_x_{noncoop}","v2_y_{noncoop}","v2_z_{noncoop}",...
%              "v1_x_{coop}","v1_y_{coop}","v1_z_{coop}",...
%              "v2_x_{coop}","v2_y_{coop}","v2_z_{coop}");
% % fontsize(lgd,22,"points");
% 
% % desired object angular velocity vs non cooperative cartesian velocities
% fig = figure("Name", "Desired object angular velocity vs non cooperative cartesian velocities");
% plot(plt.t(1:end), plt.arm1.non_coop_xdot(4:6, :),"LineWidth", 2);
% hold on;
% plot(plt.t(1:end), plt.arm2.non_coop_xdot(4:6, :) ,"LineWidth", 2);
% hold on;
% plot(plt.t(1:end), plt.feasible_coop_xdot(4:6, :) ,"LineWidth", 2);
% hold on;
% plot(plt.t(1:end), plt.feasible_coop_xdot(10:12, :) ,"LineWidth", 2);
% grid on;
% xlabel("Time [s]");
% ylabel("Velocities [m/s]");
% %fontsize(gca,22,"points");
% title("Desired object linear velocity vs non cooperative cartesian velocities");
% lgd = legend("w1_x_{noncoop}","w1_y_{noncoop}","w1_z_{noncoop}",...
%              "w2_x_{noncoop}","w2_y_{noncoop}","w2_z_{noncoop}",...
%              "w1_x_{coop}","w1_y_{coop}","w1_z_{coop}",...
%              "w2_x_{coop}","w2_y_{coop}","2_z_{coop}");
% % fontsize(lgd,22,"points");
% 

fig = figure("Name", "Component-wise velocity comparison");

% X components
subplot(3,1,1);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(1, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(1, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(1, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(7, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("X Velocity [m/s]");
title("X-Component Velocities");
legend("v1_x_{noncoop}", "v2_x_{noncoop}", "v1_x_{coop}", "v2_x_{coop}");

% Y components
subplot(3,1,2);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(2, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(2, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(2, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(8, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Y Velocity [m/s]");
title("Y-Component Velocities");
legend("v1_y_{noncoop}", "v2_y_{noncoop}", "v1_y_{coop}", "v2_y_{coop}");

% Z components
subplot(3,1,3);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(3, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(3, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(3, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(9, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Z Velocity [m/s]");
title("Z-Component Velocities");
legend("v1_z_{noncoop}", "v2_z_{noncoop}", "v1_z_{coop}", "v2_z_{coop}");


sgtitle("Comparison of Non-Cooperative and Cooperative Linear Velocities");
saveas(fig, 'CoopVsNoCoop_Linear.fig');
print(fig, 'CoopVsNoCoop_Linear', '-depsc');
% 
fig = figure("Name", "Component-wise angular velocity comparison");

% X components
subplot(3,1,1);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(4, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(4, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(4, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(10, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("X Angular Vel. [rad/s]");
title("X-Component Angular Velocities");
legend("w1_x_{noncoop}", "w2_x_{noncoop}", "w1_x_{coop}", "w2_x_{coop}");

% Y components
subplot(3,1,2);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(5, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(5, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(5, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(11, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Y Angular Vel. [rad/s]");
title("Y-Component Angular Velocities");
legend("w1_y_{noncoop}", "w2_y_{noncoop}", "w1_y_{coop}", "w2_y_{coop}");

% Z components
subplot(3,1,3);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(6, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(6, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(6, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.feasible_coop_xdot(12, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Z Angular Vel. [rad/s]");
title("Z-Component Angular Velocities");
legend("w1_z_{noncoop}", "w2_z_{noncoop}", "w1_z_{coop}", "w2_z_{coop}");

% Adjust figure layout for better readability
sgtitle("Comparison of Non-Cooperative and Cooperative Angular Velocities");
saveas(fig, 'CoopVsNoCoop_Angular.fig');
print(fig, 'CoopVsNoCoop_Angular', '-depsc');


% dist tools
fig = figure("Name", "Distance Between the Two Tools Over Time");
plot(plt.t(1:end), plt.arm1.dist_tools, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Distance between tools [m]');
title('Distance Between the Two Tools Over Time');
grid on;
saveas(fig, 'Dist_tools.fig');
print(fig, 'Dist_tools', '-depsc');

%%
%%%%%%%%%%%%%%%%%%% desiderato vs non coop  --> left
fig = figure("Name", "Component-wise linear velocity comparison");

% X components
subplot(3,1,1);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(1, :), 'LineWidth', 2);
hold on;
% plot(plt.t(1:end), plt.arm2.non_coop_xdot(1, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm1.xdot_tool(1, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm2.xdot_tool(1, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("X Velocity [m/s]");
title("X-Component Velocities");
legend("v1_x_{noncoop}", "v1_x_{des}");

% Y components
subplot(3,1,2);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(2, :), 'LineWidth', 2);
hold on;
% plot(plt.t(1:end), plt.arm2.non_coop_xdot(2, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm1.xdot_tool(2, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm2.xdot_tool(2, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Y Velocity [m/s]");
title("Y-Component Velocities");
legend("v1_y_{noncoop}", "v1_y_{des}");

% Z components
subplot(3,1,3);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(3, :), 'LineWidth', 2);
hold on;
% plot(plt.t(1:end), plt.arm2.non_coop_xdot(3, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm1.xdot_tool(3, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm2.xdot_tool(3, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Z Velocity [m/s]");
title("Z-Component Velocities");
legend("v1_z_{noncoop}", "v1_z_{des}");

sgtitle("Comparison of Non-Cooperative and Desired Linear Velocities");
saveas(fig, 'DesVsNoCoop_Linear_Left.fig');
print(fig, 'DesVsNoCoop_Linear_Left', '-depsc');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fig = figure("Name", "Component-wise angular velocity comparison - Left Arm");

% X components
subplot(3,1,1);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(4, :), 'LineWidth', 2);
hold on;
% plot(plt.t(1:end), plt.arm2.non_coop_xdot(4, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm1.xdot_tool(4, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm2.xdot_tool(4, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("X Angular Vel. [rad/s]");
title("X-Component Angular Velocities");
legend("w1_x_{noncoop}", "w1_x_{des}");

% Y components
subplot(3,1,2);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(5, :), 'LineWidth', 2);
hold on;
% plot(plt.t(1:end), plt.arm2.non_coop_xdot(5, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm1.xdot_tool(5, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm2.xdot_tool(5, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Y Angular Vel. [rad/s]");
title("Y-Component Angular Velocities");
legend("w1_y_{noncoop}","w1_y_{des}");

% Z components
subplot(3,1,3);
plot(plt.t(1:end), plt.arm1.non_coop_xdot(6, :), 'LineWidth', 2);
hold on;
% plot(plt.t(1:end), plt.arm2.non_coop_xdot(6, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm1.xdot_tool(6, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm2.xdot_tool(6, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Z Angular Vel. [rad/s]");
title("Z-Component Angular Velocities");
legend("w1_z_{noncoop}","w1_z_{des}");

% Adjust figure layout for better readability
sgtitle("Comparison of Non-Cooperative and Desired Angular Velocities - Left Arm");
saveas(fig, 'DesVsNoCoop_Angular_Left.fig');
print(fig, 'DesVsNoCoop_Angular_Left', '-depsc');



%%%%%%%%%%%%%%%%%%% desiderato vs non coop  ---> right
fig = figure("Name", "Component-wise linear velocity comparison - Right Arm");

% X components
subplot(3,1,1);
% plot(plt.t(1:end), plt.arm1.non_coop_xdot(1, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(1, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm1.xdot_tool(1, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm2.xdot_tool(1, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("X Velocity [m/s]");
title("X-Component Velocities");
legend("v2_x_{noncoop}","v2_x_{des}");

% Y components
subplot(3,1,2);
% plot(plt.t(1:end), plt.arm1.non_coop_xdot(2, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(2, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm1.xdot_tool(2, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm2.xdot_tool(2, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Y Velocity [m/s]");
title("Y-Component Velocities");
legend("v2_y_{noncoop}","v2_y_{des}");

% Z components
subplot(3,1,3);
% plot(plt.t(1:end), plt.arm1.non_coop_xdot(3, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(3, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm1.xdot_tool(3, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm2.xdot_tool(3, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Z Velocity [m/s]");
title("Z-Component Velocities");
legend("v2_z_{noncoop}", "v2_z_{des}");

sgtitle("Comparison of Non-Cooperative and Desired Linear Velocities");
saveas(fig, 'DesVsNoCoop_Linear_Right.fig');
print(fig, 'DesVsNoCoop_Linear_Right', '-depsc');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fig = figure("Name", "Component-wise angular velocity comparison - Right Arm");

% X components
subplot(3,1,1);
% plot(plt.t(1:end), plt.arm1.non_coop_xdot(4, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(4, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm1.xdot_tool(4, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm2.xdot_tool(4, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("X Angular Vel. [rad/s]");
title("X-Component Angular Velocities");
legend("w2_x_{noncoop}", "w2_x_{des}");

% Y components
subplot(3,1,2);
% plot(plt.t(1:end), plt.arm1.non_coop_xdot(5, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(5, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm1.xdot_tool(5, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm2.xdot_tool(5, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Y Angular Vel. [rad/s]");
title("Y-Component Angular Velocities");
legend("w2_y_{noncoop}", "w2_y_{des}");

% Z components
subplot(3,1,3);
% plot(plt.t(1:end), plt.arm1.non_coop_xdot(6, :), 'LineWidth', 2);
hold on;
plot(plt.t(1:end), plt.arm2.non_coop_xdot(6, :), 'LineWidth', 2);
% plot(plt.t(1:end), plt.arm1.xdot_tool(6, :), 'LineWidth', 2);
plot(plt.t(1:end), plt.arm2.xdot_tool(6, :), 'LineWidth', 2);
grid on;
xlabel("Time [s]");
ylabel("Z Angular Vel. [rad/s]");
title("Z-Component Angular Velocities");
legend("w2_z_{noncoop}", "w2_z_{des}");

% Adjust figure layout for better readability
sgtitle("Comparison of Non-Cooperative and Desired Angular Velocities");
saveas(fig, 'DesVsNoCoop_Angular_Right.fig');
print(fig, 'DesVsNoCoop_Angular_Right', '-depsc');


% 
% % dist tools
% figure;
% fig = figure("Name", "Distance Between the Two Tools Over Time");
% plot(plt.t(1:end), plt.arm1.dist_tools, 'LineWidth', 1.5);
% xlabel('Time [s]');
% ylabel('Distance between tools [m]');
% title('Distance Between the Two Tools Over Time');
% grid on;
% saveas(fig, 'Dist_tools.fig');
% print(fig, 'Dist_tools', '-depsc');



%%
fig = figure('Name', 'Comparison of Non-Cooperative and Desired Joint Velocities Left Arm', 'Position', [100, 100, 1200, 600]);

% Define colors for pairing desired and feasible velocities
colors = lines(7); % Generate a colormap with 7 distinct colors

% Subplot for first four joints
subplot(1, 2, 1);
hold on; % Enable multiple plots in the same axes
for i = 1:4
    % Plot desired velocities with dashed lines
    plot(plt.t, plt.q_dot(i, :), '--', 'Color', colors(i, :), 'LineWidth', 2); 
    % Plot feasible velocities with solid lines
    plot(plt.t, plt.arm1.feasible_q_dot(i, :), '-', 'Color', colors(i, :), 'LineWidth', 2);
end
grid on;
legend({'qdot_1 (desired)', 'qdot_1 (feasible)', ...
        'qdot_2 (desired)', 'qdot_2 (feasible)', ...
        'qdot_3 (desired)', 'qdot_3 (feasible)', ...
        'qdot_4 (desired)', 'qdot_4 (feasible)'}, ...
        'Location', 'best');
xlabel('Time [s]');
ylabel('Joint Velocity [rad/s]');
title('Arm 1: Joints 1-4');

% Subplot for last three joints
subplot(1, 2, 2);
hold on;
for i = 5:7
    % Plot desired velocities with dashed lines
    plot(plt.t, plt.q_dot(i, :), '--', 'Color', colors(i, :), 'LineWidth', 2); 
    % Plot feasible velocities with solid lines
    plot(plt.t, plt.arm1.feasible_q_dot(i, :), '-', 'Color', colors(i, :), 'LineWidth', 2);
end
grid on;
legend({'qdot_5 (desired)', 'qdot_5 (feasible)', ...
        'qdot_6 (desired)', 'qdot_6 (feasible)', ...
        'qdot_7 (desired)', 'qdot_7 (feasible)'}, ...
        'Location', 'best');
xlabel('Time [s]');
ylabel('Joint Velocity [rad/s]');
title('Arm 1: Joints 5-7');

hold off; % Stop holding the current plot

saveas(fig, 'DesVsNoCoopJointVel_Left.fig');
print(fig, 'DesVsNoCoopJointVel_Left', '-depsc');

fig = figure('Name', 'Comparison of Non-Cooperative and Desired Joint Velocities Right Arm', 'Position', [100, 100, 1200, 600]);

% Define colors for pairing desired and feasible velocities
colors = lines(7); % Generate a colormap with 7 distinct colors

% Subplot for first four joints
subplot(1, 2, 1);
hold on; % Enable multiple plots in the same axes
for i = 1:4
    % Plot desired velocities with dashed lines
    plot(plt.t, plt.q_dot2(i, :), '--', 'Color', colors(i, :), 'LineWidth', 2); 
    % Plot feasible velocities with solid lines
    plot(plt.t, plt.arm2.feasible_q_dot(i, :), '-', 'Color', colors(i, :), 'LineWidth', 2);
end
grid on;
legend({'qdot_1 (desired)', 'qdot_1 (feasible)', ...
        'qdot_2 (desired)', 'qdot_2 (feasible)', ...
        'qdot_3 (desired)', 'qdot_3 (feasible)', ...
        'qdot_4 (desired)', 'qdot_4 (feasible)'}, ...
        'Location', 'best');
xlabel('Time [s]');
ylabel('Joint Velocity [rad/s]');
title('Arm 2: Joints 1-4');

% Subplot for last three joints
subplot(1, 2, 2);
hold on;
for i = 5:7
    % Plot desired velocities with dashed lines
    plot(plt.t, plt.q_dot2(i, :), '--', 'Color', colors(i, :), 'LineWidth', 2); 
    % Plot feasible velocities with solid lines
    plot(plt.t, plt.arm2.feasible_q_dot(i, :), '-', 'Color', colors(i, :), 'LineWidth', 2);
end
grid on;
legend({'qdot_5 (desired)', 'qdot_5 (feasible)', ...
        'qdot_6 (desired)', 'qdot_6 (feasible)', ...
        'qdot_7 (desired)', 'qdot_7 (feasible)'}, ...
        'Location', 'best');
xlabel('Time [s]');
ylabel('Joint Velocity [rad/s]');
title('Arm 2: Joints 5-7');

hold off; % Stop holding the current plot

saveas(fig, 'DesVsNoCoopJointVel_Right.fig');
print(fig, 'DesVsNoCoopJointVel_Right', '-depsc');



end

