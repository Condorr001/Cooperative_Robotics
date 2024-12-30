function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            % disp('Phase 1: Reaching');
            [ang, lin] = CartError(uvms.wTv, uvms.wTgv);
            disp('Altitude: ');
            disp(uvms.altitude);
            if (norm(ang) < 0.1 && norm(lin(1:2)) < 0.1)
                
                mission.phase = 2;
                mission.phase_time = 0;
                mission.previous_action = mission.current_action;
                % mission.current_action = "landing";
                % disp('Transitioning to Phase 2: Landing');
                mission.current_action = "aligning";
                disp('Transitioning to Phase 2: Aligning');
            end
        case 2
            % disp('Phase 1: Reaching');
            disp('Theta: ');
            disp(uvms.theta);
            if (norm(uvms.theta) < 0.02)
                mission.phase = 3;
                mission.phase_time = 0;
                mission.previous_action = mission.current_action;
                mission.current_action = "landing";
                disp('Transitioning to Phase 3: Landing');
            end
        case 3
            if(uvms.altitude < 0.1)
                disp('Landed');
                mission.phase = 4;
                mission.phase_time = 0;
                mission.previous_action = mission.current_action;
                mission.current_action = "grasping";
            else
                disp("Altitude: ")
                disp(uvms.altitude);
            end
        case 4
            matrix_rock = [eye(3) uvms.rock_center; 0 0 0 1];
            [ang, lin] = CartError(matrix_rock, uvms.wTv*uvms.vTt);
            if(norm(lin) < 0.1)
                disp('DONE');
                uvms.flag = 1;
            else
                disp("Lin err: ")
                disp(norm(lin));
            end
    end
end

