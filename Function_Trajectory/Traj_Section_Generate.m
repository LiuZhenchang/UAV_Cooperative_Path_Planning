%****************************************************************************************************************************
% Discription:  Generate path segments of the UAV (From the current position to endpoint or obstacle cutting in point)
% input:        StartInfo               Starting point information
% input:        FinishInfo              Ending point information
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
% output:       TrajInfo                Matrix of path segments information
%****************************************************************************************************************************

function TrajInfo = Traj_Section_Generate(StartInfo,FinishInfo,ObsInfo,Property)
%% Plan the path segments from the current location to the endpoint
dubins_info=...                                                     % Initialize the Dubins structure based on the starting and ending point information
    Dubins_Init(StartInfo(1,:),FinishInfo(1,:));                            
TrajCollect=...                                                     % Generate all Dubins paths (LSL, RSR, RSL, LSR) that connect the starting point to the endpoint
    Dubins_Collection(dubins_info,ObsInfo,0,Property);              % Each path information is a row vector in the TrajCollect matrix
[traj_index,ObsCur]=...                                             % Select the paths that meet the requirements from all the generated dubins paths
    Dubins_Selection(TrajCollect,ObsInfo,Property,1);               % At least the path must not intersect with obstacles
[~,n1]=size(TrajCollect);                                           % Obtain the length of path information，(equals to Property.Info_length)

if traj_index(1,1)~=0                                               % If there are paths that meets the requirements
    [~,n2]=size(traj_index);                                        % Obtain the number of paths
    TrajInfo=zeros(n2,n1);                                          % Initialize the path information matrix based on the number of paths (output result)
    for i=1:n2                                                      % Traverse all paths that meet the requirements
        TrajInfo(i,:)=TrajCollect(traj_index(i),:);                 % Store the path information as row vectors in the TrajInfo matrix in sequence
        TrajInfo(i,33)=1;                                           % Set the termination flag to 1 in each path information to represent reaching the endpoint
        %Plot_Traj_Single(TrajInfo(i,:),ObsInfo,Property)
    end
    return;                                                         % End function, has reached the endpoint and has not encountered any obstacles
end                                                               


%% Plan the path from the current position to the first obstacle
flag_obs=0;                                                         % Obstacle identification
                                                                    % =1, Find obstacles that meet the requirements of path planning
                                                                    % =0, Not find obstacles that meet the requirements of path planning
while flag_obs==0
    obs_count=0;

    while ObsCur(obs_count+1)~=0                                    % Count how many obstacles are present in ObsCur
        obs_count=obs_count+1;                                      % The ObsCur records the first obstacle encountered on all paths
    end

    if obs_count==0                                                 % If there are no obstacles to reach the endpoint but the constraints are not met
        TrajInfo=zeros(1,n1);                                       % Path information output as 0
        return;                                                     % Do not perform subsequent calculations
    end
    flag_tan=zeros(1,obs_count);                                    % Initialize tangent flag of starting arc and obstacle 
    TrajTotal=zeros(obs_count*4,n1);                                % Initialize the paths total information matrix

    for i=1:obs_count                                               % Traverse every obstacle in ObsCur and plan tangent paths
        dubins_info1=dubins_info;                                   % Initialize Dubins path structure
        dubins_info1.traj.flag=1;                                   % Set the path structure for tangent path calculation
        dubins_info1.finish.xc=ObsInfo(ObsCur(1,i),1);              % Set the x-coordinate of the ending arc center to the x-coordinate of the obstacle center
        dubins_info1.finish.yc=ObsInfo(ObsCur(1,i),2);              % Set the y-coordinate of the ending arc center to the y-coordinate of the obstacle center
        dubins_info1.finish.R=ObsInfo(ObsCur(1,i),3);               % Set the radius of the ending arc to the radius of the obstacle (threat circle)
        TrajCollect=Dubins_Collection...                            % Generate all tangent paths to the current obstacle (threat circle)
            (dubins_info1,ObsInfo,ObsCur(i),Property);              % At most 4 (separated, tangent) and at least 2 (intersecting)
        TrajTotal((i-1)*4+1:i*4,:)=TrajCollect;                     % Store the generated path information in the paths total information matrix
        
        Rf_max=0;                                                   % Initialize tangent circle radius
        for j=1:2                                                   % Traverse the UAV to fly left (L) and right (R) at the initial position
            dubins_info_temp=dubins_info1;                          % Initialize a temporary Dubins path information structure
            dubins_info_temp=Dubins_Generate(dubins_info_temp,j);   % Generate path based on L or R
            xc=dubins_info_temp.start.xc;                           % Obtain the x-coordinate of the starting arc center
            yc=dubins_info_temp.start.yc;                           % Obtain the y-coordinate of the starting arc center
            Rs=dubins_info_temp.start.R;                            % Obtain the radius of the starting arc
            xo=ObsInfo(ObsCur(1,i),1);                              % Obtain the x-coordinate of the obstacle center
            yo=ObsInfo(ObsCur(1,i),2);                              % Obtain the y-coordinate of the obstacle center
            Ro=ObsInfo(ObsCur(1,i),3);
            Rf=sqrt((xc-xo)^2+(yc-yo)^2)-Rs;                        % Calculate the radius of the tangent circle
            if Rf>Rf_max                                            % Due to the different center positions of L and R, the RF is also different
                Rf_max=Rf;                                          % Select a larger tangent circle
            end
        end
        if Ro<=Rf_max                                               % If the radius of the obstacle is less than or equal to the tangent circle radius
            flag_tan(1,i)=1;                                        % Set the flag to 1, representing separation or tangency
        else                                                        % If the radius of the obstacle is greater than the radius of the tangent circle
            flag_tan(1,i)=0;                                        % Set the flag to 0, representing intersection
        end
    end

    [traj_index,ObsCur_new]=...                                     % Select the paths that meet the requirements from all the generated dubins paths
        Dubins_Selection(TrajTotal,ObsInfo,Property,2);             % 1: The path does not intersect with obstacles
                                                                    % 2: The turning angle of the path shall not exceed 3 * pi/2
                                                                    % 3: Simultaneously satisfying 1 and 2
    if traj_index(1)~=0||sum(flag_tan)==0                           % If there are avaliable paths or if compression of the threat circle is required
        flag_obs=1;                                                 % Set obstacle flag as 1 to terminate the loop
    else                                                            % Other conditions
        ObsCur=ObsCur_new;                                          % Update the current obstacle information and continue the loop
    end                                                             % Until the tangent path between the drone and the obstacle is found and there are no other obstacles on that path

end


%% There is no available flight path, and the radius of the threat circle needs to be compressed
if traj_index==0                                                    % If there is no available path, the UAV must enter the threat circle
    Property.invasion=1;                                            % Set the intrusion threat circle flag to 1
    for i=1:obs_count                                               % Traverse every obstacle

        Rf_max=0;                                                   % Initialize tangent circle radius
        for j=1:2                                                   % Traverse the UAV to fly left (L) and right (R) at the initial position
            dubins_info_temp=dubins_info;                           % Initialize a temporary Dubins path information structure
            dubins_info_temp=Dubins_Generate(dubins_info_temp,j);   % Generate path based on L or R
            xc=dubins_info_temp.start.xc;                           % Obtain the x-coordinate of the starting arc center
            yc=dubins_info_temp.start.yc;                           % Obtain the y-coordinate of the starting arc center
            Rs=dubins_info_temp.start.R;                            % Obtain the radius of the starting arc
            xo=ObsInfo(ObsCur(1,i),1);                              % Obtain the x-coordinate of the obstacle center
            yo=ObsInfo(ObsCur(1,i),2);                              % Obtain the y-coordinate of the obstacle center
            Rf=sqrt((xc-xo)^2+(yc-yo)^2)-Rs;                        % Calculate the radius of the compressed obstacle（threat circle）
            if Rf>Rf_max                                            % Due to the different center positions of L and R, the RF is also different
                Rf_max=Rf;                                          % Select a larger tangent circle
            end                                                     % The depth (radius compression) at which UAVs enter the threat circle should be as small as possible
        end
        
        ObsInfo(ObsCur(1,i),3)=Rf_max*0.99;                         % Update the compressed obstacle radius in ObsInfo
        dubins_info1=dubins_info;                                   % Initialize Dubins path structure
        dubins_info1.traj.flag=1;                                   % Set the path structure for tangent path calculation
        dubins_info1.finish.xc=ObsInfo(ObsCur(1,i),1);              % Set the x-coordinate of the ending arc center to the x-coordinate of the obstacle center
        dubins_info1.finish.yc=ObsInfo(ObsCur(1,i),2);              % Set the y-coordinate of the ending arc center to the y-coordinate of the obstacle center
        dubins_info1.finish.R=Rf_max;                               % Set the radius of the ending arc to the radius of the obstacle (threat circle)
        
        TrajCollect=Dubins_Collection...                            % Generate all tangent paths to the current obstacle (threat circle)
            (dubins_info1,ObsInfo,ObsCur(i),Property);              % After radius compression, there will be four paths when two circles are tangent
        TrajTotal((i-1)*4+1:i*4,:)=TrajCollect;                     % Store the generated path information in the paths total information matrix
    end
    [traj_index,~]=...                                              % % Select the paths that meet the requirements from all the generated dubins paths
        Dubins_Selection(TrajTotal,ObsInfo,Property,2);             % 1: The path does not intersect with obstacles
                                                                    % 2: The turning angle of the path shall not exceed 3 * pi/2                                                                    % 3: Simultaneously satisfying 1 and 2
end

if traj_index==0
    TrajInfo=zeros(1,n1);
else
    [~,n2]=size(traj_index);                                        % Obtain the number of paths
    TrajInfo=zeros(n2,n1);                                          % Initialize the path information matrix based on the number of paths (output result)
    for i=1:n2                                                      % Traverse all paths that meet the requirements
        TrajInfo(i,:)=TrajTotal(traj_index(i),:);                   % Store the path information as row vectors in the TrajInfo matrix in sequence
        %Plot_Traj_Single(TrajInfo(i,:),ObsInfo,Property,0)
    end
end

end

