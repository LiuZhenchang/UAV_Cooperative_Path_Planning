clear;
clc;
addpath('Function_Plot');
addpath('Function_Dubins');
addpath('Function_Trajectory');

%% Initialize Data
Property.obs_last=0;                                                % Record the obstacles avoided during current trajectory planning
Property.invasion=0;                                                % Record whether there is any intrusion into obstacles (threat areas) during trajectory planning
Property.mode=2;                                                    % Set trajectory generation mode 1: shortest path; 2: Conventional path
Property.ns=50;                                                     % Set the number of discrete points in the starting arc segment
Property.nl=50;                                                     % Set the number of discrete points in the straight line segment
Property.nf=50;                                                     % Set the number of discrete points at the end of the arc segment
Property.max_obs_num=5;                                             % Set the maximum number of obstacles to be detected for each path planning
Property.max_info_num=20;                                           % Set the maximum number of stored path segments for each planning step
Property.max_step_num=4;                                            % Set the maximum number of planned steps for the path
Property.Info_length=33;                                            % Set the length of each path information
Property.radius=100*1e3;                                            % Set the turning radius of the UAV（mm）
Property.scale=1/1000;
Property.increment=20*1e3;                                          % Set the adjustment range of path lenth increment
Property.selection1=3;                                              % Set path filtering mode 1
Property.selection2=1;                                              % Set path filtering mode 2
                                                                    % =1: The path does not intersect with obstacles
                                                                    % =2: The turning angle of the path shall not exceed 3 * pi/2
                                                                    % =3: Simultaneously satisfying 1 and 2
% Set starting point infomation
StartInfo=[ 50*1e3,     30*1e3,     0,          100*1e3;            % positon x, position y, yaw angle, starting arc radius
            80*1e3,     -40*1e3,    pi/6,       100*1e3;            % xs, ys, phi_s, R_s
            20*1e3,     80*1e3,     -pi/6,      100*1e3];           % unit (mm)

% Set ending point information
FinishInfo=[550*1e3,    120*1e3,    0,      100*1e3;                % positon x, position y, yaw angle, starting arc radius
            500*1e3,    100*1e3,    0,      100*1e3;                % xf, yf, phi_f, R_f
            500*1e3,    140*1e3,    0,      100*1e3];               % unit (mm)

% Set obastacles (threat circle) information
ObsInfo=[   150*1e3,    50*1e3,     50*1e3;                         % positon x, position y, threat circle radius
            300*1e3,    100*1e3,    20*1e3;                         % xo, yo, Ro
            400*1e3,    50*1e3,     50*1e3;                         % unit (mm)
            200*1e3,    200*1e3,    50*1e3];


[uav_num,~]=size(StartInfo);                                        % Obtain UAVs number
[obs_num,~]=size(ObsInfo);                                          % Obtain obstacles number

Coop_State(1:uav_num)=struct(...                                    % The structure of flight paths information for UAVs
    'traj_length',[],...                                            % Array of all path length 
    'traj_length_max',0,...                                         % Maximum path length
    'traj_length_min',0,...                                         % Minimum path length
    'TrajSeqCell',[],...                                            % Path sequence cell array
    'ideal_length',12*1e5,...                                       % Expected path length
    'optim_length',0,...                                            % Optimized path length
    'traj_index_top',0,...                                          % Index of path that lenth is greater than and closest to the expected path length
    'traj_index_bottom',0,...                                       % Index of path that lenth is shorter than and closest to the expected path length
    'TrajSeq_Coop',[]);                                             % Matrix of cooperative path sequence


%% Plan the path of each UAV from the starting point to the endpoint in sequence
for uav_index=1:3                                                   % Traverse each UAV
    start_info=StartInfo(uav_index,:);                              % Obtain the starting point information of the UAV
    finish_info=FinishInfo(uav_index,:);                            % Obtain the ending point information of the UAV
    Property.radius=start_info(4);                                  % Set the turning radius of the UAV based on initial information
    TrajSeqCell=Traj_Collection...                                  % Calculate all available flight paths for the UAV
        (start_info,finish_info,ObsInfo,Property);                  
    Coop_State(uav_index)=Coop_State_Update...                      % Select the basic path from the available flight paths
        (TrajSeqCell,Coop_State(uav_index),ObsInfo,Property);       % and optimize the basic path to generate a cooperative path

    Plot_Traj_Multi_Modification(TrajSeqCell,ObsInfo,Property);
end

Plot_Traj_Coop(Coop_State,ObsInfo,Property,1,1);                    % Plot cooperative path planning results
