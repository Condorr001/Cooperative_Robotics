function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Compute distance between tools for plotting
pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));

% Compute minimum altitude reference ALWAYS
% gains to control each arm behaviour
gainL = 1;
gainR = 1;

min_alt = 0.15;
pandaArm.ArmL.xdot.alt = min_alt - pandaArm.ArmL.altitude;
pandaArm.ArmR.xdot.alt = min_alt - pandaArm.ArmL.altitude;

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
pandaArm.ArmL.xdot.jl = gainL * (((pandaArm.jlmax + pandaArm.jlmin) / 2) - pandaArm.ArmL.q);
pandaArm.ArmR.xdot.jl = gainR * (((pandaArm.jlmax + pandaArm.jlmin) / 2) - pandaArm.ArmR.q);

switch mission.phase
    case 1 % going into grasp position
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArm.ArmL.wTg, pandaArm.ArmL.wTt);
        pandaArm.ArmL.xdot.tool = gainL * [ang, lin];

        % limit the requested velocities, by checking the datasheet for
        % maximum allowed velocities for the joints
        pandaArm.ArmL.xdot.tool(1:3) = Saturate(pandaArm.ArmL.xdot.tool(1:3), deg2rad(150)); %limit on angular velocity, which is 150 °/s from the datasheet
        pandaArm.ArmL.xdot.tool(4:6) = Saturate(pandaArm.ArmL.xdot.tool(4:6), 2); %limit on linear velocity, which is 2m/s from datasheet

        % RIGHT ARM -> same as left arm, as they're two equal Franka
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArm.ArmR.wTg, pandaArm.ArmR.wTt);
        pandaArm.ArmR.xdot.tool = gainR * [ang, lin];

        % limit the requested velocities...
        pandaArm.ArmR.xdot.tool(1:3) = Saturate(pandaArm.ArmR.xdot.tool(1:3), deg2rad(150));
        pandaArm.ArmR.xdot.tool(4:6) = Saturate(pandaArm.ArmR.xdot.tool(4:6), 2); 

    case 2 % grasping
        % Perform the rigid grasp of the object and move it

        % COMMON
        % -----------------------------------------------------------------
        % Rigid Grasp Constraint
        pandaArm.xdot.rc = zeros(6, 1); % I pass a vector of null linear and angular velocity

        % LEFT ARM
        % -----------------------------------------------------------------        
        % Object position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTog, pandaArm.ArmL.wTo);
        pandaArm.ArmL.xdot.tool = [ang, lin];

        % limit the requested velocities...
        pandaArm.ArmL.xdot.tool(1:3) = Saturate(pandaArm.ArmL.xdot.tool(1:3), deg2rad(150)); %limit on angular velocity, which is 150 °/s from the datasheet
        pandaArm.ArmL.xdot.tool(4:6) = Saturate(pandaArm.ArmL.xdot.tool(4:6), 2); %limit on linear velocity, which is 2m/s from datasheet

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Object position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTog, pandaArm.ArmR.wTo);
        pandaArm.ArmR.xdot.tool = [ang, lin];

        % limit the requested velocities...
        pandaArm.ArmR.xdot.tool(1:3) = Saturate(pandaArm.ArmR.xdot.tool(1:3), deg2rad(150));
        pandaArm.ArmR.xdot.tool(4:6) = Saturate(pandaArm.ArmR.xdot.tool(4:6), 2); 
    case 3
        % Stop any motions
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.ArmL.xdot.tool(1:3) = zeros(3,1);
        pandaArm.ArmL.xdot.tool(4:6) = zeros(3,1);

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.ArmR.xdot.tool(1:3) = zeros(3,1);
        pandaArm.ArmR.xdot.tool(4:6) = zeros(3,1);
        
end


