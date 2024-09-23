%*********************************************************************************************************************
% Discription:  Generate the modified path based on the primary path and radius increments 
% input:        TrajSeq                 Matrix of primary path sequence
% input:        Increment               Increments of the starting circle and the ending circle of path segments 
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
% output:       TrajSeq_new             Matrix of modified path sequence
% output:       flag                    flag of path generation results
%*********************************************************************************************************************

function [TrajSeq_new,flag] = Traj_Seq_Modification(TrajSeq,Increment,ObsInfo,Property)

[dubins_num,clm]=size(TrajSeq);                                     % Obtain the number of Dubins path segments
[~,increm_num]=size(Increment);                                     % Obtain the number of increments
if dubins_num*2~=increm_num                                         % Determine whether the number of increments is twice the number of path segments (rows)
    error("increment size does not match")
end
TrajSeq_new=zeros(dubins_num,clm);                                  % Initialize the matrix of modified path sequence
flag=2;                                                             % Initialize flag
                                                                    % =0, No avaliable path
                                                                    % =1，The path intersects with obstacles
                                                                    % =2，Has avaliable path
%% Modify each path segment sequentially based on increments information
for i=1:dubins_num                                                  
    obs_index=TrajSeq(i,25);                                        % Obtain the avoidance obstacle of the current path
    type=TrajSeq(i,1);                                              % Obtain the type of Dubins path
    start_info=zeros(1,4);                                          % Initialize starting point info
    finish_info=zeros(1,4);                                         % Initialize ending point info
    %% Set starting point info
    if i==1                                                         % Set the starting point of the first path segment
        start_info(1)=TrajSeq(i,2);                                 % Set the starting point x coordinate
        start_info(2)=TrajSeq(i,3);                                 % Set the starting point y coordinate
        start_info(3)=TrajSeq(i,4);                                 % Set starting heading angle
        start_info(4)=TrajSeq(i,5)+Increment(i*2-1);                % Set starting arc radius
    else                                                            % Set the starting point based on the endpoint of the previous path segment
        start_info(1)=TrajSeq_new(i-1,13);                          % Set the starting point x coordinate
        start_info(2)=TrajSeq_new(i-1,14);                          % Set the starting point y coordinate
        start_info(3)=TrajSeq_new(i-1,15);                          % Set starting heading angle
        if Property.radius<TrajSeq(i-1,16)                          % If the turning radius of the UAV is smaller than the radius of the obstacle
            start_info(4)=TrajSeq(i-1,16);                          % Set obstacle's radius as starting arc radius
        else                                                        % If the turning radius of the UAV is larger than the radius of the obstacle
            start_info(4)=Property.radius;                          % Set UAV's radius as starting arc radius
        end
        start_info(4)=start_info(4)+Increment(i*2-1);               % Adjust the starting arc radius with increment
    end
    %% Set ending point info and path segment info
    if i==dubins_num                                                % Set the ending point of the last path segment
        finish_info(1)=TrajSeq(i,13);                               % Set the ending point x coordinate
        finish_info(2)=TrajSeq(i,14);                               % Set the ending point y coordinate
        finish_info(3)=TrajSeq(i,15);                               % Set ending heading angle
        finish_info(4)=TrajSeq(i,16)+Increment(i*2);                % Set ending arc radius
        dubins_info=Dubins_Init(start_info,finish_info);            % Initial Dubins path info structure
    else                                                            % Set the ending point of tangent path based on the target obstacle
        dubins_info=Dubins_Init(start_info,finish_info);            % Initial Dubins path info structure
        dubins_info.traj.flag=1;                                    % Set path type as tangent path
        dubins_info.finish.xc=ObsInfo(obs_index,1);                 % Set x coordinate of ending arc center  
        dubins_info.finish.yc=ObsInfo(obs_index,2);                 % Set y coordinate of ending arc center
        dubins_info.finish.R=ObsInfo(obs_index,3)+Increment(i*2);   % Set ending arc radius
    end
    %% Generate and save Dubins paths
    dubins_info=Dubins_Generate(dubins_info,type);                  % Calculate Dubins path info
    if dubins_info.traj.length==0                                   % If the path length is 0, it indicates that there is no avaliable path segment
        flag=0;                                                     % Set path generate flag to 0
        return;
    end
    Property.obs_last=TrajSeq(i,25);                                % Update the obstacle number that needs to be avoided in Property
    Property.invasion=TrajSeq(i,32);                                % Update whether the path allows intrusion into the threat area in Property
    ObsSeries=Dubins_Obs_Check(dubins_info,ObsInfo,Property);       % Perform obstacle detection on the current path segment
    if ObsSeries(1,1)~=0                                            % if path intesect with obstacles
        flag=1;                                                     % Set path generate flag to 1
    end
    TrajInfo=Traj_Info_Array(dubins_info,ObsSeries,Property);       % Store path info and obstacle info into an array
    TrajInfo(1,25)=obs_index;                                       % Supplement obstacle number
    TrajSeq_new(i,:)=TrajInfo(1,:);                                 % Store the current path segment info into path sequence matrix


end

end

