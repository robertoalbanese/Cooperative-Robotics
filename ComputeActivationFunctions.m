function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
%uvms.A.t = eye(6);  /old
uvms.A.v = eye(6);
