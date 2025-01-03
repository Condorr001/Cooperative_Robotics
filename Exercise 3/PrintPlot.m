function [ ] = PrintPlot( plt)

fig = figure('Name', 'Joint position and velocity Left Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
grid on;
title('LEFT ARM');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
grid on;
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');

fig = figure('Name', 'Joint position and velocity Right Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q2);
grid on;
title('RIGHT ARM');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot2);
grid on;
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');

%% Activation functions
% Left arm
fig = figure("Name","Left Arm Activation Function");
plot(plt.t(1:end), plt.arm1.a_jl(1,:), plt.t(1:end), plt.arm1.a_ma, plt.t(1:end), plt.arm1.a_tool(1,:), plt.t(1:end), plt.a_rc(1,:), "LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Activation");
%fontsize(gca,22,"points");
title("Left Arm Activation Function");
%lgd = legend("MA","T","RC","Location","northeastoutside");
lgd = legend("JL", "MA","T","RC");
%fontsize(lgd,22,"points");
ylim([-0.1 1.1]);

% Right arm
fig = figure("Name","Right Arm Activation Function");
plot(plt.t(1:end), plt.arm2.a_jl(1,:), plt.t(1:end), plt.arm2.a_ma, plt.t(1:end), plt.arm2.a_tool(1,:), plt.t(1:end), plt.a_rc(1,:), "LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Activation");
%fontsize(gca,22,"points");
title("Right Arm Activation Function");
%lgd = legend("MA","T","RC","Location","northeastoutside");
lgd = legend( "JL", "MA","T","RC");
%fontsize(lgd,22,"points");
ylim([-0.1 1.1]);

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

% Adjust figure layout for better readability
sgtitle("Comparison of Non-Cooperative and Cooperative Linear Velocities");

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


end

