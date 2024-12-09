function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                [angL, linL] = CartError(pandaArm.ArmL.wTg, pandaArm.ArmL.wTt);
                [angR, linR] = CartError(pandaArm.ArmR.wTg, pandaArm.ArmR.wTt);
                
                % max error: 1/10 cm and 1deg
                if(norm(angL) <= deg2rad(1) && norm(linL) <= 0.001 && norm(angR) <= deg2rad(1) && norm(linR) <= 0.001)
                     mission.phase = 2;
                     mission.phase_time = 0;                     
                     mission.prev_action = mission.current_action;
                     mission.current_action = "coop_manip";                 
                end

                disp("Moving");
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task
                [angL, linL] = CartError(pandaArm.wTog, pandaArm.ArmL.wTo);                
                [angR, linR] = CartError(pandaArm.wTog, pandaArm.ArmR.wTo);

                % max error: 1 cm and 3deg
                if(norm(angL) <= deg2rad(3) && norm(linL) <= 0.01 && norm(angR) <= deg2rad(3) && norm(linR) <= 0.01)
                     mission.phase = 3;                     
                     mission.phase_time = 0;
                     mission.prev_action = mission.current_action;                     
                     mission.current_action = "end_motion";
                end
               
                disp("Grabbing");

            case 3 % Finish motion
                disp("Finished");
        end
end

