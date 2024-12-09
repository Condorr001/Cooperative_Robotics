%{
function [ ] = PrintPlot( plt, pandaArms )

% some predefined plots
% you can add your own

%% Left Arm
fig = figure("Name","Left Arm Joint Limits Activation");
plot(plt.t(1:end), plt.armL.a_jl,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Join Limits Activation", "FontSize", 24);
fontsize(gca,22,"points");
title(["Left Arm Activation Function", "Joint Limits"], "FontSize",24);
lgd = legend("Aj1","Aj2","Aj3","Aj4","Aj5","Aj6","Aj7","Location","northeastoutside");
fontsize(lgd,22,"points");


fig = figure("Name","Left Arm Activations");
plot(plt.t(1:end), plt.armL.a_ma,...
    plt.t(1:end), plt.armL.a_tool(1,:), ...
    plt.t(1:end), plt.a_rc(1,:), ...
    "LineWidth",2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Activations", "FontSize", 24);
fontsize(gca,22,"points");
title("Left Arm Activation Function", "FontSize",24);
lgd = legend("MA","T","RC","Location","northeastoutside");
fontsize(lgd,22,"points");
ylim([-0.1 1.1]);


fig = figure("Name", "Left Arm Desired cartesian velocities");
subplot(2,1,1);
plot(plt.t(1:end), plt.armL.xdot_tool,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Velocities [m/s] [rad/s]", "FontSize", 24);
fontsize(gca,22,"points");
title("Left Arm Desired Cartesian Velocities in <w>", "FontSize",24);
lgd = legend("v_x","v_y","v_z","omega_x","omega_y","omega_z","Location","northeastoutside");
fontsize(lgd,22,"points");


fig = figure("Name", "Left Arm Cartesian velocities");
subplot(2,1,1);
plot(plt.t(1:end), plt.armL.xdot,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Velocities [m/s] [rad/s]", "FontSize", 24);
fontsize(gca,22,"points");
title("Left Arm Cartesian Velocities in <w>", "FontSize",24);
lgd = legend("v_x","v_y","v_z","omega_x","omega_y","omega_z","Location","northeastoutside");
fontsize(lgd,22,"points");


%% Right Arm
fig = figure("Name","Right Arm Joint Limits Activation");
plot(plt.t(1:end), plt.armR.a_jl,"LineWidth", 2);
grid on;
xlabel("Time [sec]");
ylabel("Join Limits Activation");
fontsize(gca,22,"points");
title(["Right Arm Activation Function", "Joint Limits"], "FontSize",24);
lgd = legend("Aj1","Aj2","Aj3","Aj4","Aj5","Aj6","Aj7","Location","northeastoutside");
fontsize(lgd,22,"points");


fig = figure("Name","Right Arm Activations");
plot(plt.t(1:end), plt.armR.a_ma,...
    plt.t(1:end), plt.armR.a_tool(1,:), ...
    plt.t(1:end), plt.a_rc(1,:), ...              % plt.a_rc(1,:)
    "LineWidth",2);
grid on;
xlabel("Time [s]", "FontSize",24);
ylabel("Activations", "FontSize", 24);
fontsize(gca,22,"points");
title("Right Arm Activation Function", "FontSize",24);
lgd = legend("MA","T","RC","Location","northeastoutside");
fontsize(lgd,22,"points");
ylim([-0.1 1.1]);


fig = figure("Name", "Right Arm Desired cartesian velocities");
subplot(2,1,1);
plot(plt.t(1:end), plt.armL.xdot_tool,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Velocities [m/s] [rad/s]", "FontSize", 24);
fontsize(gca,22,"points");
title("Right Arm Desired Cartesian Velocities in <w>", "FontSize",24);
lgd = legend("v_x","v_y","v_z","omega_x","omega_y","omega_z","Location","northeastoutside");
fontsize(lgd,22,"points");


fig = figure("Name", "Right Arm Cartesian velocities");
subplot(2,1,2);
plot(plt.t(1:end), plt.armR.xdot,"LineWidth", 2);
grid on;
xlabel("Time [sec]", "FontSize",24);
ylabel("Velocities [m/s] [rad/s]", "FontSize", 24);
fontsize(gca,22,"points");
title("Right Arm Cartesian Velocities in <w>", "FontSize",24);
lgd = legend("v_x","v_y","v_z","omega_x","omega_y","omega_z","Location","northeastoutside");
fontsize(lgd,22,"points");


end
%}

function [ ] = PrintPlot( plt, pandaArms )

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
plot(plt.t(1:end), plt.armL.a_ma, plt.t(1:end), plt.armL.a_tool(1,:), plt.t(1:end), plt.a_rc(1,:), "LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Activation");
%fontsize(gca,22,"points");
title("Left Arm Activation Function");
%lgd = legend("MA","T","RC","Location","northeastoutside");
lgd = legend("MA","T","RC");
%fontsize(lgd,22,"points");
ylim([-0.1 1.1]);

% Right arm
fig = figure("Name","Right Arm Activation Function");
plot(plt.t(1:end), plt.armR.a_ma, plt.t(1:end), plt.armR.a_tool(1,:), plt.t(1:end), plt.a_rc(1,:), "LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Activation");
%fontsize(gca,22,"points");
title("Right Arm Activation Function");
%lgd = legend("MA","T","RC","Location","northeastoutside");
lgd = legend("MA","T","RC");
%fontsize(lgd,22,"points");
ylim([-0.1 1.1]);

%% Desired vs end-effector angular velocities
% Left arm
fig = figure("Name", "Left Arm Desired vs End-Effector Angular Velocities");
plot(plt.t(1:end), plt.armL.xdot_tool(1:3, :),"LineWidth", 2);
hold on;
plot(plt.t(1:end), plt.armL.xdot(1:3, :) ,"LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Velocities [rad/s]");
%fontsize(gca,22,"points");
title("Left Arm Desired vs End-Effector Angular Velocities");
lgd = legend("w_x_{des}","w_y_{des}","w_z_{des}","w_x_{ee}","w_y_{ee}","w_z_{ee}");
%fontsize(lgd,22,"points");

% Right arm
fig = figure("Name", "Right Arm Desired vs End-Effector Angular Velocities");
plot(plt.t(1:end), plt.armR.xdot_tool(1:3, :),"LineWidth", 2);
hold on;
plot(plt.t(1:end), plt.armR.xdot(1:3, :) ,"LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Velocities [rad/s]");
%fontsize(gca,22,"points");
title("Right Arm Desired vs End-Effector Angular Velocities");
lgd = legend("w_x_{des}","w_y_{des}","w_z_{des}","w_x_{ee}","w_y_{ee}","w_z_{ee}");
%fontsize(lgd,22,"points");

%% Desired vs end-effector linear velocities
% Left arm
fig = figure("Name", "Left Arm Desired vs End-Effector Linear Velocities");
plot(plt.t(1:end), plt.armL.xdot_tool(4:6, :),"LineWidth", 2);
hold on;
plot(plt.t(1:end), plt.armL.xdot(4:6, :) ,"LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Velocities [m/s]");
%fontsize(gca,22,"points");
title("Left Arm Desired vs End-Effector Linear Velocities");
lgd = legend("v_x_{des}","v_y_{des}","v_z_{des}","v_x_{ee}","v_y_{ee}","v_z_{ee}");
%fontsize(lgd,22,"points");

% Right arm
fig = figure("Name", "Right Arm Desired vs End-Effector Linear Velocities");
plot(plt.t(1:end), plt.armR.xdot_tool(4:6, :),"LineWidth", 2);
hold on;
plot(plt.t(1:end), plt.armR.xdot(4:6, :) ,"LineWidth", 2);
grid on;
xlabel("Time [s]");
ylabel("Velocities [m/s]");
%fontsize(gca,22,"points");
title("Right Arm Desired vs End-Effector Linear Velocities");
lgd = legend("v_x_{des}","v_y_{des}","v_z_{des}","v_x_{ee}","v_y_{ee}","v_z_{ee}");
%fontsize(lgd,22,"points");



end