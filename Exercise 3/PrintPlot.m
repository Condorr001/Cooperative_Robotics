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
lgd = legend("w_x_des","w_y_des","w_z_des","w_x_ee","w_y_ee","w_z_ee");
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
lgd = legend("w_x_des","w_y_des","w_z_des","w_x_ee","w_y_ee","w_z_ee");
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
lgd = legend("v_x_des","v_y_des","v_z_des","v_x_ee","v_y_ee","v_z_ee");
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
lgd = legend("v_x_des","v_y_des","v_z_des","v_x_ee","v_y_ee","v_z_ee");
%fontsize(lgd,22,"points");


end

