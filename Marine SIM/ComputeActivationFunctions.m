  function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1  
            uvms.Aa.v = eye(6);
            uvms.Aa.ha = eye(1);
            uvms.Aa.ma = eye(1);
            uvms.Aa.t = zeros(6);
            
            [ang_v, lin_v] = CartError(uvms.wTgv , uvms.wTv);
            if (norm(lin_v)<0.1)
                mission.phase = 2;
                mission.phase_time = 0;
            end
        case 2
            uvms.Aa.t = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time)*eye(6);
            uvms.Aa.v = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time)*eye(6);
            uvms.Aa.ha = eye(1);
            uvms.Aa.ma = eye(1);
    end
% arm tool position control
% always active
uvms.A.t = eye(6) * uvms.Aa.t;

% vehicle-frame position control
uvms.A.v = eye(6) * uvms.Aa.v;

%horizontal attitude
uvms.A.ha = IncreasingBellShapedFunction(0.05, 0.1 , 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha;
%uvms.A.ha = 1;

%minimum altitude
uvms.A.ma = DecreasingBellShapedFunction(1, 1.2, 0, 1, uvms.sensorDistance) * uvms.Aa.ma;