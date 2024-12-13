function [pandaArm, pandaArm2, mission] = UpdateMissionPhase(pandaArm, pandaArm2, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                [ang, lin] = CartError(pandaArm.wTg, pandaArm.wTt);

                % max error: 1/10 cm and 1deg
                if(norm(ang) <= deg2rad(1) && norm(lin) <= 0.001)
                     mission.phase = 2;
                     mission.phase_time = 0;                     
                     mission.prev_action = mission.current_action;
                     mission.current_action = "coop_manip";                 
                end

                disp("Moving");
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task
                [ang, lin] = CartError(pandaArm.wTog, pandaArm.wTo);

                fprintf('ang=%f\n', ang);
                fprintf('lin=%f\n', lin);

                % max error: 1 cm and 3deg
                if(norm(ang) <= deg2rad(3) && norm(lin) <= 0.01)
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

