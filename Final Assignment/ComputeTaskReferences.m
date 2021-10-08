function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here
gain = 0.6;
sat = 0.6;
% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = gain * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), sat);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), sat);

% reference for vehicle position control task
[v_ang, v_lin] = CartError(uvms.wTgv , uvms.wTv);
uvms.xdot.v = gain * [v_ang; v_lin];
% limit the requested velocities...
uvms.xdot.v(1:6) = Saturate(uvms.xdot.v(1:6), sat);

% under actuated control task 
uvms.xdot.ua = 1 * (uvms.p_dot);

% horizontal attitude control task
uvms.xdot.ha = Saturate(gain *(0 - norm(uvms.v_rho)), sat);

% minimum distance from the floor control task
uvms.xdot.md = Saturate(gain * (uvms.altitude), sat);

% landing control task
uvms.xdot.l = Saturate(gain * (0.03 - uvms.altitude), sat);

% rock alignment control task
%uvms.xdot.ra = (1/(uvms.altitude+0.01)) * 0.06 * (norm(uvms.v_rock_vehicle));

% zero velocity contraint task
uvms.xdot.zv = zeros(6,1);

% joint limit control task
uvms.xdot.jlim = Saturate(gain * ((uvms.jlmin + uvms.jlmax)/2) - uvms.q, sat);

%==================================================================================================================================%
%--------------------------------------------------Dexrov--------------------------------------------------------------------------%
%==================================================================================================================================%

% joint fixed position control task
q_des = [-0.0031 1.2586 0.0128 -1.2460]';
uvms.xdot.jfp = gain * ( q_des - uvms.q(1:4));




