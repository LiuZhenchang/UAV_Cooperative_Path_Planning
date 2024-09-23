%****************************************************************************************************************************
% Discription:  Discretize Dubins path into waypoints sequence
% input:        dubins_info             Complete Dubins path information
% input:        ns                      Number of discrete points in the starting arc
% input:        ns                      Number of discrete points in the straight line
% input:        nf                      Number of discrete points in the ending arc
% output:       dubibs_x                Array of waypoints x coordinate
% output:       dubibs_y                Array of waypoints y coordinate
%****************************************************************************************************************************

function [dubins_x,dubins_y] = Dubins_Discret(dubins_info,ns,nl,nf)

% With the determination of the starting and ending positions and velocity directions, 
% there are a total of four types of dubins paths
% (1) LSL (Left Straight Left),  (2) RSR (Right Straight Right)
% (3) LSR (Left Straight Right), (4) RSL (Right Straight Left)

circle_centre_start_param = [-1, 1,-1, 1];                  %¡¡-1(L), 1(R)
circle_centre_finish_param =[-1, 1, 1,-1];                  %¡¡-1(L), 1(R)
param_s=circle_centre_start_param(dubins_info.traj.type);   % Starting arc center calculation parameters
param_f=circle_centre_finish_param(dubins_info.traj.type);  % Ending arc center calculation parameters

%% Discretize starting arc into waypoints sequence
xc_s=dubins_info.start.xc;                                  % Starting arc center x coordinate
yc_s=dubins_info.start.yc;                                  % Starting arc center y coordinate
R_s=dubins_info.start.R;                                    % Starting arc radius
phi_sc=dubins_info.start.phi_c;                             % Starting point azimuth angle
% phi_ex=dubins_info.start.phi_ex;                          % Cutting out azimuth angle
psi_s=dubins_info.start.psi;                                % travel angle on the starting arc
if psi_s==0
    phi_s_temp=phi_sc;
else
    d_phi_s=-param_s*psi_s/ns;                              % Calculate the discrete angle size of the starting arc
    phi_s_temp=phi_sc:d_phi_s:phi_sc-param_s*psi_s;         % Discretize travel angle on the starting arc
end

dubins_xs=xc_s+R_s*cos(phi_s_temp);                         % Calculate waypoints x coordinate sequence on starting arc
dubins_ys=yc_s+R_s*sin(phi_s_temp);                         % Calculate waypoints y coordinate sequence on starting arc

%% Discretize ending arc into waypoints sequence
xc_f=dubins_info.finish.xc;                                 % Ending arc center x coordinate
yc_f=dubins_info.finish.yc;                                 % Ending arc center y coordinate
R_f=dubins_info.finish.R;                                   % Ending arc radius
% phi_fc=dubins_info.finish.phi_c;                          % Ending point azimuth angle
phi_en=dubins_info.finish.phi_en;                           % Cutting in azimuth angle
psi_f=dubins_info.finish.psi;                               % travel angle on the ending arc
if psi_f==0
    phi_f_temp=phi_en;
else
    d_phi_f=-param_f*psi_f/nf;                              % Calculate the discrete angle size of the ending arc
    phi_f_temp=phi_en:d_phi_f:phi_en-param_f*psi_f;         % Discretize travel angle on the ending arc
end
dubins_xf=xc_f+R_f*cos(phi_f_temp);                         % Calculate waypoints x coordinate sequence on ending arc
dubins_yf=yc_f+R_f*sin(phi_f_temp);                         % Calculate waypoints y coordinate sequence on ending arc

%% Discretize straight line into waypoints sequence
x_ex=dubins_info.start.x_ex;                                % Cutting out point x coordinate
y_ex=dubins_info.start.y_ex;                                % Cutting out point y coordinate
x_en=dubins_info.finish.x_en;                               % Cutting in point x coordinate
y_en=dubins_info.finish.y_en;                               % Cutting in point y coordinate
if x_en==x_ex&&y_en==y_ex                                   % In condition the length of the straight line is 0
    dubins_xl=x_ex;
    dubins_yl=y_ex;
elseif x_en==x_ex&&y_en~=y_ex                               % In condition the straight line is is perpendicular to the x-axis
    dubins_yl=y_ex:(y_en-y_ex)/nl:y_en;
    [~,m]=size(dubins_yl);
    dubins_xl=zeros(1,m)+x_en;
elseif x_en~=x_ex&&y_en==y_ex                               % In condition the straight line is is perpendicular to the y-axis
    dubins_xl=x_ex:(x_en-x_ex)/nl:x_en;
    [~,m]=size(dubins_xl);
    dubins_yl=zeros(1,m)+y_en;
else
    dubins_xl=x_ex:(x_en-x_ex)/nl:x_en;                     % Calculate waypoints x coordinate sequence on straight line
    dubins_yl=y_ex:(y_en-y_ex)/nl:y_en;                     % Calculate waypoints y coordinate sequence on straight line
end

%% Synthetic total waypoints
dubins_x=[dubins_xs,dubins_xl,dubins_xf];
dubins_y=[dubins_ys,dubins_yl,dubins_yf];
end

