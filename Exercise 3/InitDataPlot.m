function [plt] = InitDataPlot( maxloops)

    plt.t = zeros(1, maxloops);
    
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);
    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);
    
    % LEFT ARM

    % activation joint limits
    plt.arm1.a_jl = zeros(7, maxloops);
    % activation minimum altitude 
    plt.arm1.a_ma = zeros(1, maxloops);
    % activation tool 
    plt.arm1.a_tool = zeros(6, maxloops);
    % desired cartesian velocity
    plt.arm1.xdot_tool = zeros(6, maxloops);
    % cartesian velocity
    plt.arm1.xdot = zeros(6, maxloops);

    % RIGHT ARM

    % activation joint limits 
    plt.arm2.a_jl = zeros(7, maxloops);
    % activation minimum altitude 
    plt.arm2.a_ma = zeros(1, maxloops);
    % activation tool 
    plt.arm2.a_tool = zeros(6, maxloops);
    % desired cartesian velocity
    plt.arm2.xdot_tool = zeros(6, maxloops);
    % cartesian velocity
    plt.arm2.xdot = zeros(6, maxloops);

    % activation rigid constraint
    plt.a_rc = zeros(6, maxloops);

    % mission phase
    plt.phase = zeros(1, maxloops);


end

