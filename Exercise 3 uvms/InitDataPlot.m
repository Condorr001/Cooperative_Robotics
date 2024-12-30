function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);

    plt.p = zeros(6, maxloops);
    plt.p_dot = zeros(6, maxloops);
    
    % activation functions
    plt.A.ma = zeros(1, maxloops);
    plt.A.ha = zeros(1, maxloops);
    plt.A.vp = zeros(3, maxloops);
    plt.A.vo = zeros(3, maxloops);
    plt.A.zvc = zeros(6, maxloops);
    plt.A.hal = zeros(1, maxloops);
    plt.A.g = zeros(6, maxloops);
    plt.A.lan = zeros(1, maxloops);

    % Desired vel
    plt.xdot_des = zeros(6, maxloops); 
    % Real vel
    plt.xdot = zeros(6, maxloops);

    plt.a = zeros(11, maxloops);

    % Vehicle Position and Orientation
    plt.vpos = zeros(3, maxloops);
    plt.vorient = zeros(3, maxloops);
    
    % Real Vehicle Position and Orientation
    plt.vpos_dot = zeros(3, maxloops);
    plt.vorient_dot = zeros(3, maxloops);

    % Desired Vehicle Position and Orientation
    plt.vpos_dot_des = zeros(3, maxloops);
    plt.vorient_dot_des = zeros(3, maxloops);

end

