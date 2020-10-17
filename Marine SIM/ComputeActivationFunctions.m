function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle-frame position control
uvms.A.v = eye(6);

%horizontal attitude
%uvms.A.ha = IncreasingBellShapedFunction(0.05, 0.1 , 0, 1, norm(uvms.v_rho));
uvms.A.ha = 1;

%minimum altitude
uvms.A.a = DecreasingBellShapedFunction(1, 1.2, 0, 1, uvms.sensorDistance);