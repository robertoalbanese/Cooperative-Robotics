function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1            
            
            [uvms.ang_v, uvms.lin_v] = CartError(uvms.wTgv , uvms.wTv);
            if (norm(uvms.lin_v)<0.1)
                mission.phase = 2;
                mission.phase_time = 0;
            end
        case 2
    end
end

