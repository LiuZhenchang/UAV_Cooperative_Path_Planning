%****************************************************************************************************************************
% Discription:  Generate path information for all types (LSL, RSR, LSR, RSL) based on basic Dubins path information
% input:        dubins_info             Basic Dubins path information
% input:        ObsInfo                 Matrix of obstacles information
% input:        obs_index               The number of the obstacle to be avoided currently
% input:        Property                Structure of path planning parameters
% output:       TrajCollect             Matrix of all type paths information
%****************************************************************************************************************************

function TrajCollect = Dubins_Collection(dubins_info,ObsInfo,obs_index,Property)
TrajCollect=zeros(4,Property.Info_length);                          % Initialize the matrix of all type paths information
for type=1:4                                                        % Traverse each type of Dubins path
    dubins_info=Dubins_Generate(dubins_info,type);                  % Generate complete path information based on basic path information and path type
    if dubins_info.traj.length~=0                                   % If the generated trajectory length is not 0,it indicates the existence of a trajectory
        ObsSeries=Dubins_Obs_Check(dubins_info,ObsInfo,Property);   % Perform obstacle detection on the current path
    else
        continue;
    end
    TrajInfo=Traj_Info_Array(dubins_info,ObsSeries,Property);       % Record the Dubins path information and obstacle information on the path into an array
    TrajInfo(1,25)=obs_index;                                       % Record the number of the current obstacle to be avoided
    TrajCollect(type,:)=TrajInfo(1,:);                              % Store all types of path information
end

end

