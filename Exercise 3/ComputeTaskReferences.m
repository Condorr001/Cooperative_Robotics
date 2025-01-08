function [pandaArm] = ComputeTaskReferences(pandaArm,mission)

% % % % % % % % % Compute distance between tools for plotting
% % % % % % % % pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));

% gain to control the arm behaviour
gain = 1;

% Compute minimum altitude reference ALWAYS
min_alt = 0.15;
pandaArm.xdot.alt = gain * (min_alt - pandaArm.altitude);

% % % % % % % % % take the smallest value, that is what matters most
% % % % % % % % % pandaArm.min_alt = min(alt_L, alt_R);
pandaArm.min_alt = min_alt;

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
pandaArm.xdot.jl = gain * (((pandaArm.jlmax + pandaArm.jlmin) / 2) - pandaArm.q);

switch mission.phase
    case 1
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTg, pandaArm.wTt);

        pandaArm.xdot.tool = [gain*ang; gain*lin];
        % Limits request velocities
        pandaArm.xdot.tool(1:3) = Saturate(pandaArm.xdot.tool(1:3), deg2rad(150));
        pandaArm.xdot.tool(4:6) = Saturate(pandaArm.xdot.tool(4:6), 2);  
    case 2
        % Object position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTog, pandaArm.wTo);
       
        pandaArm.xdot.tool = [gain*ang; gain*lin];
        % Limits request velocities
        pandaArm.xdot.tool(1:3) = Saturate(pandaArm.xdot.tool(1:3), deg2rad(150));
        pandaArm.xdot.tool(4:6) = Saturate(pandaArm.xdot.tool(4:6), 2);  

    case 3
        % Stop any motions
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.xdot.tool(1:3) = zeros(3,1);
        pandaArm.xdot.tool(4:6) = zeros(3,1);

end



