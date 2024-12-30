function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');


figure(2);
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');


% figure(3);
% hplot = plot(plt.t, plt.a(1:7,:));
% set(hplot, 'LineWidth', 2);
% legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');
% 
% figure(4);
% hplot = plot(plt.t, plt.a(8:9,:));
% set(hplot, 'LineWidth', 2);
% legend('Amu', 'Aha');
    
% All Activation Functions 
figure(3), grid on;
title( "Activation Functions", "All The Activation Functions" );
hold on;
plot(plt.t, plt.A.zvc, 'Color', [0.850, 0.325, 0.098]); % Rust Red
plot(plt.t, plt.A.ma, 'Color', [0.929, 0.694, 0.125]); % Mustard Yellow
plot(plt.t, plt.A.ha, 'Color', [0.466, 0.674, 0.188]); % Olive Green
plot(plt.t, plt.A.lan, 'Color', [0.301, 0.745, 0.933]); % Light Blue
plot(plt.t, plt.A.hal, 'Color', [0.635, 0.078, 0.184]); % Burgundy
plot(plt.t, plt.A.vp, 'Color', [0.125, 0.466, 0.705]); % Sky Blue
plot(plt.t, plt.A.vo, 'Color', [0.984, 0.607, 0.062]); % Orange
plot(plt.t, plt.A.g, 'Color', [0.494, 0.184, 0.556]); % Purple
hold off;
ylim( [0, 1.1] )          
legend( ...
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
    'grasping' )

    

end

