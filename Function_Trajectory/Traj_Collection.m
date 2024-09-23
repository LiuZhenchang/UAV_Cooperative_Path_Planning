%****************************************************************************************************************************
% Discription:  Plan obstacle avoidance paths for each UAV based on starting and ending information
% input:        StartInfo               Starting point information
% input:        FinishInfo              Ending point information
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
% output:       TrajSeqCell             Cell Array of avaliable paths
%****************************************************************************************************************************

function TrajSeqCell = Traj_Collection(StartInfo,FinishInfo,ObsInfo,Property)
%% Initialize information
TrajInfoCell=cell(1,Property.max_step_num);                         % Initialize database to store path segments infomation
for i=1:Property.max_step_num
    TrajInfoCell{1,i}=...                                           % Each cell stores a matrix
        zeros(Property.max_info_num,Property.Info_length);          % Each row vector of the matrix represents a path segment result
end
step=1;                                                             % Initialize the number of path planning steps
TrajFlag=0;                                                         % Initialize the termination flag of path planning loop

%% Segmented planning of obstacle avoidance path, stopping planning after reaching the endpoint
while TrajFlag~=1
    if step==1                                                      % If it is the first step of path planning, the starting point is the initial position
        TrajInfo=Traj_Section_Generate...                           % Plan the path segment that meets the constraint conditions (reaching the endpoint or tangency with obstacles)
            (StartInfo,FinishInfo,ObsInfo,Property);                % The constraints are set by Property
        [traj_num,~]=size(TrajInfo);                                % Obtain the number of generated path segments
        for i=1:traj_num                                            % Traverse all generated path segments under the current step
            TrajInfoCell{1,step}(i,:)=TrajInfo(i,:);                % Store the path information into the row vectors of the matrix of TrajInfoCell
            if TrajInfo(i,33)==1                                    % If the termination information of the path is 1 (indicating that the endpoint has been reached)
                TrajFlag=1;                                         % Set the termination flag of path planning loop to 1
            end
        end
    else                                                            % If it is not for the first step of path planning
        %% Obtain the previous path planning result
        TrajInfoTemp=...                                            % Initialize temporary path information storage matrix
            zeros(Property.max_info_num*2,Property.Info_length);    % Store all paths planned under the current step
        traj_count=0;                                               % Initialize the path counter
        TrajPast=TrajInfoCell{1,step-1};                            % Obtain the path segments information matrix planned in the previous step
        [n,~]=size(TrajPast);                                       % Obtain the row number of the path segments information matrix
        traj_past_num=0;                                            % Initialize the number of planned path segments in the previous step
        for i=1:n                                                   % Traverse each row of the path segments information matrix
            if TrajPast(i,1)==0                                     % If the variable at the beginning of the row is 0
                break;                                              % Stop loop
            end
            traj_past_num=traj_past_num+1;                          % Count the number of planned path segments in the previous step
        end
        %% Traverse all previously planned path segments and plan a new path based on the previous segments endpoint
        for i=1:traj_past_num                                       % Traverse each path segments planned in the previous step
            if TrajPast(i,24)==0||TrajPast(i,33)==1                 % If the previous path segment length is 0 or has already reached the endpoint
                continue;                                           % Skip this path and no longer continue planning
            end
            % Set the starting point positon and heading angle of the current path segment 
            % to the ending point positon and heading angle of the previous path segment 
            StartInfo(1)=TrajPast(i,13);                            % Set starting point x coordinate;
            StartInfo(2)=TrajPast(i,14);                            % Set starting point y coordinate;
            StartInfo(3)=TrajPast(i,15);                            % Set starting deading angle
            if TrajPast(i,25)~=0                                    % If the previous path segment was an obstacle avoidance path
                if Property.radius<TrajPast(i,16)                   % If the turning radius of the UAV is smaller than the radius of the obstacle
                    StartInfo(4)=TrajPast(i,16);                    % Set the obstacle radius as the starting radius
                else                                                % If the turning radius of the UAV is larger than the radius of the obstacle
                    StartInfo(4)=Property.radius;                   % Set the turning radius of the UAV as the starting radius
                end
            end
            Property.obs_last=TrajPast(i,25);                       % Update the number of obstacle intersect with the previous path segments in Property
            Property.invasion=TrajPast(i,32);                       % Update whether the previous oath segment has invaded obstacles in Property
            TrajInfo=Traj_Section_Generate...                       % Plan the path segment that meets the constraint conditions (reaching the endpoint or tangency with obstacles)）
                (StartInfo,FinishInfo,ObsInfo,Property);            % The constraints are set by Property
            [traj_num,~]=size(TrajInfo);                            % Obtain the number of generated path segments
            for j=1:traj_num                                        % Traverse all generated path segments under the current step
                if TrajInfo(j,1)==0                                 % If the variable at the beginning of the row is 0
                    continue;                                       % It means that no results that meet the constraints have been planned and skip the result
                end
                traj_count=traj_count+1;                            % Count the number of planned path segments
                TrajInfoTemp(traj_count,:)=TrajInfo(j,:);           % Store the path segments information as row vectors in the psth information matrix
                if TrajInfo(j,33)==1&&Property.mode==1              % If the path segemnt has reached the end point and it is the shortest path mode
                    TrajFlag=1;                                     % Set the termination flag of path planning loop to 1
                end
            end
        end
        %% Screen path segments for storage
        if traj_count>Property.max_info_num                         % If the number of generated path segments is greater than the maximum storage number
            traj_count=Property.max_info_num;                       % truncated storage
        end
        for i=1:traj_count                                          % Traverse each row of the temporary path segments information matrix
            TrajInfoCell{1,step}(i,:)=TrajInfoTemp(i,:);            % Save path segments in database
        end
    end
    step=step+1;                                                    % Update the number of steps for path planning
    if step>Property.max_step_num                                   % If the step number is greater than the maximum step number
        TrajFlag=1;                                                 % Stop Loop 
    end
end
%% Generate all flight path sequence and stored in TrajSeqCell
TrajSeqCell=Traj_Seq_Generate(TrajInfoCell);                        % In TrajSeqCell, each cell stores a matrix of path sequence
                                                                    % The row numbers of the matrix correspond to the number of steps in path planning, 
                                                                    % and the row vectors represent the generated path segments under the corresponding steps
end

