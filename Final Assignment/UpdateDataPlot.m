function [ plt ] = UpdateDataPlot( plt, uvms, t, loop, mission )

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.tool_err(:,loop) = abs(uvms.goalPosition - uvms.ee_pos);
%plt.mission_phase(:,loop) = mission.phase;

plt.tool_goal (:,loop) = uvms.goalPosition;
plt.ee_pos(:,loop) = uvms.ee_pos;

plt.rock_aling(:,loop) = norm(uvms.v_rho_rock);

plt.vel(:,loop) = uvms.p_dot(3);

plt.lin_threshold(:,loop) = norm(uvms.lin_v(1:2));

plt.toolPos(:, loop) = uvms.wTt(1:3,4);

plt.ha_mis(:,loop) = norm(uvms.v_rho);

plt.alt(:,loop) = uvms.altitude;

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;

%plt.altitude(loop) = uvms.altitude;

%plt.xdot_jl(:, loop) = uvms.xdot.jl;
%plt.xdot_mu(:, loop) = uvms.xdot.mu;
plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;

plt.a(1:7, loop) = diag(uvms.A.jl);
plt.a(8, loop) = uvms.A.mu;
plt.a(9, loop) = uvms.A.ha(1,1);
plt.a(10, loop) = uvms.A.md;

plt.toolx(:,loop) = uvms.wTt(1,4);
plt.tooly(:,loop) = uvms.wTt(2,4);
end