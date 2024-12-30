function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

% joints
figure(1); 
subplot(2,1,1); grid on;
hplot = plot(plt.t, plt.q);
title( "Joint position","FontSize",14,"FontName",'Arial');
set(hplot, 'LineWidth', 1);
legendObj = legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
legendObj.Title.String = 'Legend';
xlabel('Time (s)','FontSize', 12, 'FontName', 'Arial');
ylabel('Angle (rad)', 'FontSize', 12, 'FontName', 'Arial');

subplot(2,1,2); grid on;
hplot = plot(plt.t, plt.q_dot);
title( "Joint velocity","FontSize",14,"FontName",'Arial');
set(hplot, 'LineWidth', 1);
legendObj = legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
legendObj.Title.String = 'Legend';
xlabel('Time (s)','FontSize', 12, 'FontName', 'Arial'); 
ylabel('Velocity (rad/s)', 'FontSize', 12, 'FontName', 'Arial');

% vehicle
figure(2); grid on;
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
title( "Vehicle position","FontSize",14,"FontName",'Arial');
set(hplot, 'LineWidth', 2);
legendObj = legend('x','y','z','roll','pitch','yaw');
legendObj.Title.String = "Legend";
xlabel('Time (s)','FontSize', 12, 'FontName', 'Arial'); 
ylabel('Position (m)', 'FontSize', 12, 'FontName', 'Arial');

subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
title( "Vehicle velocity","FontSize",14,"FontName",'Arial');
set(hplot, 'LineWidth', 2);
legendObj = legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');
legendObj.Title.String = 'Legend';
xlabel('Time (s)','FontSize', 12, 'FontName', 'Arial'); 
ylabel('Velocity (m/s)', 'FontSize', 12, 'FontName', 'Arial');
    
% All Activation Functions 
figure(3), grid on;
title( "Activation Functions","FontSize",17,"FontName",'Arial');
hold on;
plot(plt.t, plt.A.zvc, 'Color', [0.850, 0.325, 0.098], 'LineWidth', 2); % Rust Red
plot(plt.t, plt.A.ma, 'Color', [0.929, 0.694, 0.125],'LineWidth', 2); % Mustard Yellow
plot(plt.t, plt.A.ha, 'Color', [0.466, 0.674, 0.188],'LineWidth', 2); % Olive Green
plot(plt.t, plt.A.lan, 'Color', [0.301, 0.745, 0.933], 'LineWidth', 2); % Light Blue
plot(plt.t, plt.A.hal, 'Color', [0.635, 0.078, 0.184], 'LineWidth', 2); % Burgundy
plot(plt.t, plt.A.vp, 'Color', [0.125, 0.466, 0.705], 'LineWidth', 2); % Sky Blue
plot(plt.t, plt.A.vo, 'Color', [0.984, 0.607, 0.062], 'LineWidth', 2); % Orange
plot(plt.t, plt.A.g, 'Color', [0.494, 0.184, 0.556], 'LineWidth', 2); % Purple
hold off;
ylim([0, 1.1]);  
yticks([0 1]); 
legendObj = legend( ...
    'zero velocity constraint', ...
    'minimum altitude', ...
    'horizontal attitude', ...
    'landing', ...
    'vehicle horizontal alignment to the target', ...
    'vehicle position control -- x', ...
    '-- y', ...
    '-- z', ...
    'vehicle orientation control -- roll', ...
    '-- pitch', ...
    '-- yaw', ...
    'grasping' );

legendObj.Title.String = 'Legend';
xlabel('Time (s)','FontSize', 12, 'FontName', 'Arial'); 
ylabel('Activated Function', 'FontSize', 12, 'FontName', 'Arial');
    

% figure(3);
% hplot = plot(plt.t, plt.a(1:7,:));
% set(hplot, 'LineWidth', 2);
% legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');
% 
% figure(4);
% hplot = plot(plt.t, plt.a(8:9,:));
% set(hplot, 'LineWidth', 2);
% legend('Amu', 'Aha');

end

