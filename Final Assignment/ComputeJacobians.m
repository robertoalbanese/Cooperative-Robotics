function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

% vehicle position control task
uvms.Jv = [ zeros(3,7) zeros(3,3) uvms.wTv(1:3,1:3);
            zeros(3,7) uvms.wTv(1:3,1:3) zeros(3,3)];

% horizontal attitude control task 
w_kw = [0 0 1]';
v_kv = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;

uvms.v_rho = ReducedVersorLemma(v_kw, v_kv);
v_n = uvms.v_rho/norm(uvms.v_rho);

uvms.Jha = [zeros(1,7) zeros(1,3) v_n'];

% minimum distance from the floor control task
uvms.altitude = w_kw' * uvms.wTv(1:3,1:3) * [0 0 uvms.sensorDistance]';
%uvms.Jmd = [zeros(1,7) 0 0 1 zeros(1,3)];
uvms.Jmd = w_kw' * [ zeros(3,7) uvms.wTv(1:3,1:3) zeros(3,3)];
% landing control task
uvms.Jl = [zeros(1,7) 0 0 1 zeros(1,3)];

% alignment to rock frame control task
v_iv = [1 0 0]';

proj_matrix = [1 0 0; 0 1 0; 0 0 0];
rock_center = [12.2025   37.3748  -39.8860 1]'; % in world frame homogeneous coordinates
v_rock_center = uvms.vTw * rock_center; %distance vector of rock from vehicle frame projected on world

v_or = proj_matrix * v_rock_center(1:3);
uvms.v_rho_rock = ReducedVersorLemma(v_iv, v_or);

if (norm(uvms.v_rho_rock) > 0) % avoid division by zero
uvms.v_n_rock = uvms.v_rho_rock/norm(uvms.v_rho_rock);
else
uvms.v_n_rock = [0 0 0]';
end
uvms.Jra = [zeros(1,7) zeros(1,3) uvms.v_n_rock']; %rock alignment

% zero velocity contraint task
%uvms.Jzv = [zeros(6,7) eye(6)];
uvms.Jzv =[ zeros(3,7) zeros(3,3) uvms.wTv(1:3,1:3);
            zeros(3,7) uvms.wTv(1:3,1:3) zeros(3,3)];

%joint limit control task
uvms.Jjlim = [eye(7,7) zeros(7,6)];

%==================================================================================================================================%
%--------------------------------------------------Dexrov--------------------------------------------------------------------------%
%==================================================================================================================================%

% joint fixed position control task
uvms.Jjfp = [eye(4) zeros(4,3) zeros(4,6)];


% under actuated control task
uvms.Jua = [zeros(6,7) eye(6)];

end