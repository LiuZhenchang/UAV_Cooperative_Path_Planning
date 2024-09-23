%****************************************************************************************************************************
% Discription:  Generate complete path information based on basic path information and path type
% input:        dubins_info             Basic Dubins path information
% input:        tupe                    Dubins path type
% output:       dubins_info             Complete Dubins path information
%****************************************************************************************************************************

function dubins_info = Dubins_Generate(dubins_info,type)
%% Initialize Dubins path information
dubins_info.traj.type=type;                                 % Set path type 
dubins_info.traj.erro=0;                                    % Reset error marker

%% Calculation parameters of the Dubins path
% Please refer to Eq.(1) and Table 1 in Paper "Multi-UAV Cooperative Path-planning Under Complex Threat Environment" for details
circle_centre_start_param = [-1, 1,-1, 1];
circle_centre_finish_param =[-1, 1, 1,-1];
exit_point_angle_param_1 =  [-1, 1, 1,-1];
enter_point_angle_param_1 = [-1, 1, 1,-1];
exit_point_angle_param_2 =  [3*pi/2, pi/2, -pi/2, pi/2];
enter_point_angle_param_2 = [3*pi/2, pi/2, pi/2,  3*pi/2];
start_rotate_angle =        [-1, 1,-1, 1];
finish_rotate_angle =       [-1, 1, 1,-1];
tangent_type =              [-1,-1, 1, 1];

%% Calculation information of starting and ending arc centers
x_s=dubins_info.start.x;                                    % Obtain starting point x coordinate
y_s=dubins_info.start.y;                                    % Obtain starting point y coordinate
R_s=dubins_info.start.R;                                    % Obtain starting arc radius
phi_s=dubins_info.start.phi;                                % Obtain starting heading angle
param_s=circle_centre_start_param(type);                    % Obtain starting arc center param
xc_s=x_s-R_s*cos(phi_s+param_s*pi/2);                       % Calculater starting arc center x coordinate
yc_s=y_s-R_s*sin(phi_s+param_s*pi/2);                       % Calculater starting arc center y coordinate
dubins_info.start.phi_c=phi_s+param_s*pi/2;                 % Set starting point azimuth angle
dubins_info.start.xc=xc_s;                                  % Set starting arc center x coordinate
dubins_info.start.yc=yc_s;                                  % Set starting arc center y coordinate

if dubins_info.traj.flag==0                                 % Calculate Dubins path information
    x_f=dubins_info.finish.x;                               % Obtain ending point x coordinate
    y_f=dubins_info.finish.y;                               % Obtain ending point y coordinate
    R_f=dubins_info.finish.R;                               % Obtain ending arc radius
    phi_f=dubins_info.finish.phi;                           % Obtain ending heading angle
    param_f=circle_centre_finish_param(type);               % Obtain ending arc center param
    xc_f=x_f-R_f*cos(phi_f+param_f*pi/2);                   % Calculater ending arc center x coordinate
    yc_f=y_f-R_f*sin(phi_f+param_f*pi/2);                   % Calculater ending arc center y coordinate
    dubins_info.finish.phi_c=phi_f+param_f*pi/2;            % Set ending point azimuth angle
    dubins_info.finish.xc=xc_f;                             % Set ending arc center x coordinate
    dubins_info.finish.yc=yc_f;                             % Set ending arc center y coordinate

elseif dubins_info.traj.flag==1                             % Calculate Tangent path information
    xc_f=dubins_info.finish.xc;                             % Obtain ending arc center x coordinate
    yc_f=dubins_info.finish.yc;                             % Obtain ending arc center y coordinate
    R_f=dubins_info.finish.R;                               % Obtain ending arc radius
end

%% Calculate cutting out and cutting in azimuth angles
param_t=tangent_type(type);
param_ex1=exit_point_angle_param_1(type);
param_ex2=exit_point_angle_param_2(type);
param_en1=enter_point_angle_param_1(type);
param_en2=enter_point_angle_param_2(type);

c=sqrt((xc_s-xc_f)^2+(yc_s-yc_f)^2);                        % Calculate distance between starting arc center and ending arc center
if param_t==1&&(R_s+R_f)>c||...                             % Determine whether there is a feasible solution for the Dubins path
        param_t==-1&&abs(R_f-R_s)>c
    dubins_info.traj.erro=1;                                % Set error marker
    dubins_info.traj.length=0;                              % Set the path length to 0 in the absence of feasible solutions
    %warning('No Dubins Trajectory Exist\n')                % Output error info
    return;                                                 % terminate function
end
alpha=asin((R_f+param_t*R_s)/c);                            % Calculate the included angle between the line of the circle centers and the straight line
beta=atan2(yc_f-yc_s,xc_f-xc_s);                            % Calculate the included angle between the line of the circle centers and the x-axis.
phi_ex=beta+param_ex1*alpha+param_ex2;                      % Calculate cutting out azimuth angle
phi_en=beta+param_en1*alpha+param_en2;                      % Calculate cutting in azimuth angle
phi_en1=beta+param_en1*alpha;                               % Calculate cutting in heading angle
dubins_info.start.phi_ex=phi_ex;                            % Set cutting out azimuth angle
dubins_info.finish.phi_en=phi_en;                           % Set cutting in azimuth angle
dubins_info.finish.phi_en1=phi_en1;                         % Set cutting in heading angle

%% Calculate cutting out and cutting in points
x_ex=xc_s+R_s*cos(phi_ex);                                  % Calculate cutting out point x coordinate
y_ex=yc_s+R_s*sin(phi_ex);                                  % Calculate cutting out point y coordinate
x_en=xc_f+R_f*cos(phi_en);                                  % Calculate cutting in point x coordinate
y_en=yc_f+R_f*sin(phi_en);                                  % Calculate cutting in point y coordinate

dubins_info.start.x_ex=x_ex;                                % Set cutting out point x coordinate
dubins_info.start.y_ex=y_ex;                                % Set cutting out point y coordinate
dubins_info.finish.x_en=x_en;                               % Set cutting in point x coordinate
dubins_info.finish.y_en=y_en;                               % Set cutting in point y coordinate

%% Calculate travel angles on starting and ending arcs
param_rs=start_rotate_angle(type);
psi_s=mod(pi/2+param_rs*(phi_s-phi_ex),2*pi);               % Calculate travel angle on the starting arc
dubins_info.start.psi=psi_s;                                % Set travel angle on the starting arc

if dubins_info.traj.flag==0                                 % Calculate Dubins path information
    param_rf=finish_rotate_angle(type);
    psi_f=mod(-pi/2+param_rf*(-phi_f+phi_en),2*pi);         % Calculate travel angle on the ending arc
    if abs(psi_f)<3.1416*2&&abs(psi_f)>3.1415*2             % If the absolute value of psi_f is near the 2 * pi
        psi_f=0;                                            % Set psi_f to 0
    end
    dubins_info.finish.psi=psi_f;                           % Set travel angle on the ending arc

elseif dubins_info.traj.flag==1                             % Calculate Tangent path information
    psi_f=0;
    dubins_info.finish.psi=psi_f;                           % Set travel angle on the ending arc to 0
    dubins_info.finish.phi=phi_en1;                         % Set ending heading angle to cutting in heading angle
    dubins_info.finish.x=x_en;                              % Set ending point x coordinate
    dubins_info.finish.y=y_en;                              % Set ending point y coordinate
end

%% Calculate path length
cs=R_s*psi_s;                                               % Calculate starting arc length
l=sqrt((x_en-x_ex)^2+(y_en-y_ex)^2);                        % Calculate straight line length
cf=R_f*psi_f;                                               % Calculate ending arc length
dubins_info.traj.length=cs+l+cf;                            % Set path length
end


