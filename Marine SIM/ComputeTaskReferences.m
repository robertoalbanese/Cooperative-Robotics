function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% reference for vehicle position control task
[ang_v, lin_v] = CartError(uvms.wTgv , uvms.wTv);
uvms.xdot.v = 0.2 * [ang_v; lin_v];
% limit the requested velocities...
uvms.xdot.v(1:3) = Saturate(uvms.xdot.v(1:3), 0.2);
uvms.xdot.v(4:6) = Saturate(uvms.xdot.v(4:6), 0.2);

%horizontal attitude
uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho));

% minimum altitude
uvms.xdot.a = 0.2 * uvms.sensorDistance;


