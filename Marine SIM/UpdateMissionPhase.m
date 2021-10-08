function [uvms, mission] = UpdateMissionPhase(uvms, mission)
mission.phase_time
    switch mission.phase
        case 1              
            [ang_v, lin_v] = CartError(uvms.wTgv , uvms.wTv);
            if (norm(lin_v)<0.1)
                mission.phase = 2;
                mission.phase_time = 0;
            end
        case 2
    end
end

