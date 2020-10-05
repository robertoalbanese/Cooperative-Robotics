function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
[ang, lin] = CartError(uvms.wTg , uvms.wTv);

uvms.xdot.t = 0.2 * [ang; lin];
uvms.xdot.v = 0.2 * [ang; lin];

% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);
uvms.xdot.v(1:3) = Saturate(uvms.xdot.v(1:3), 0.2);
uvms.xdot.v(4:6) = Saturate(uvms.xdot.v(4:6), 0.2);

