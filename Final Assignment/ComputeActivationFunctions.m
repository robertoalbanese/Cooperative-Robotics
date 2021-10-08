function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
if mission.enabled == 1
    switch mission.phase
        case 1  
            uvms.Aa.v = eye(6);
            uvms.Aa.ha = eye(1);
            uvms.Aa.md = eye(1);
            uvms.Aa.l = zeros(1);
            uvms.Aa.t = zeros(6);
            uvms.Aa.ra = zeros(1);
            uvms.Aa.zv = eye(13);
        case 2
            uvms.Aa.t = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time)*eye(6);
            uvms.Aa.v = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time)*eye(6);
            uvms.Aa.l = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.ha = eye(1);
            uvms.Aa.md = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.ra = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.zv = eye(13);
    end
else
    uvms.Aa.v = eye(6);
    uvms.Aa.ha = eye(1);
    uvms.Aa.md = eye(1);
    uvms.Aa.l = eye(1);
    uvms.Aa.t = eye(6);
    uvms.Aa.ra = eye(1);
    uvms.Aa.zv = eye(13);
end
% arm tool position control task
% always active
uvms.A.t = eye(6) * uvms.Aa.t;

% vehicle position control task
uvms.A.v = eye(6) * uvms.Aa.v;

% under actuated control task
uvms.A.ua = diag([0 0 0 0 1 0]);

% horizontal attitude control task
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2 , 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha;
%uvms.A.ha = 1;

% minimum distance from the floor control task
uvms.A.md = DecreasingBellShapedFunction(1, 2, 0, 1, uvms.altitude) * uvms.Aa.md;

% landing control task
uvms.A.l = 1 * uvms.Aa.l;

% rock alignment control task
uvms.A.ra = 1 * uvms.Aa.ra;

%zero velocity control task
uvms.A.zv = eye(6) * uvms.Aa.zv;

% joint limit control task
act_joint1_min = DecreasingBellShapedFunction(uvms.jlmin(1), (uvms.jlmin(1) + uvms.jlmin(1)/5) , 0, 1, uvms.q(1));
act_joint2_min = DecreasingBellShapedFunction(uvms.jlmin(2), (uvms.jlmin(2) + uvms.jlmin(2)/5) , 0, 1, uvms.q(2));
act_joint3_min = DecreasingBellShapedFunction(uvms.jlmin(3), (uvms.jlmin(3) + uvms.jlmin(3)/5) , 0, 1, uvms.q(3));
act_joint4_min = DecreasingBellShapedFunction(uvms.jlmin(4), (uvms.jlmin(4) + uvms.jlmin(4)/5) , 0, 1, uvms.q(4));
act_joint5_min = DecreasingBellShapedFunction(uvms.jlmin(5), (uvms.jlmin(5) + uvms.jlmin(5)/5) , 0, 1, uvms.q(5));
act_joint6_min = DecreasingBellShapedFunction(uvms.jlmin(6), (uvms.jlmin(6) + uvms.jlmin(6)/5) , 0, 1, uvms.q(6));
act_joint7_min = DecreasingBellShapedFunction(uvms.jlmin(7), (uvms.jlmin(7) + uvms.jlmin(7)/5) , 0, 1, uvms.q(7));

act_joint1_max = IncreasingBellShapedFunction(uvms.jlmax(1), (uvms.jlmax(1) + uvms.jlmax(1)/5) , 0, 1, uvms.q(1));
act_joint2_max = IncreasingBellShapedFunction(uvms.jlmax(2), (uvms.jlmax(2) + uvms.jlmax(2)/5) , 0, 1, uvms.q(2));
act_joint3_max = IncreasingBellShapedFunction(uvms.jlmax(3), (uvms.jlmax(3) + uvms.jlmax(3)/5) , 0, 1, uvms.q(3));
act_joint4_max = IncreasingBellShapedFunction(uvms.jlmax(4), (uvms.jlmax(4) + uvms.jlmax(4)/5) , 0, 1, uvms.q(4));
act_joint5_max = IncreasingBellShapedFunction(uvms.jlmax(5), (uvms.jlmax(5) + uvms.jlmax(5)/5) , 0, 1, uvms.q(5));
act_joint6_max = IncreasingBellShapedFunction(uvms.jlmax(6), (uvms.jlmax(6) + uvms.jlmax(6)/5) , 0, 1, uvms.q(6));
act_joint7_max = IncreasingBellShapedFunction(uvms.jlmax(7), (uvms.jlmax(7) + uvms.jlmax(7)/5) , 0, 1, uvms.q(7));
uvms.A.jlim = diag([act_joint1_min + act_joint1_max,act_joint2_min + act_joint2_max,act_joint3_min + act_joint3_max,act_joint4_min + act_joint4_max,act_joint5_min + act_joint5_max,act_joint6_min + act_joint6_max,act_joint7_min + act_joint7_max]);

%==================================================================================================================================%
%--------------------------------------------------Dexrov--------------------------------------------------------------------------%
%==================================================================================================================================%

% joint fixed position control task
uvms.A.jfp = eye(4);