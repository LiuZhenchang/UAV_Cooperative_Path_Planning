%****************************************************************************************************************************
% Discription:  Update cooperative flight state for the UAV
% input:        TrajSeqCell             Cell array of available paths for the UAV
% input:        state                   Structure of flight paths information for the UAV
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
% output:       state                   Structure of flight paths information for the UAV
%****************************************************************************************************************************

function State = Coop_State_Update(TrajSeqCell,State,ObsInfo,Property)

[~,n]=size(TrajSeqCell);                                    % Obtain the number of paths
State.traj_length=zeros(n,1);                               % Initialize the array of path length
State.TrajSeqCell=TrajSeqCell;                              % Save cell array of available paths for the UAV

%% Obtain path information
for i=1:n                                                   % Traverse each path
   length=Traj_Length(TrajSeqCell{1,i});                    % Calculate path length
   State.traj_length(i,1)=length;                           % Save path length

   %% Obtain the longest and shortest flight paths
   if i==1                                                  % If it is the first path
       State.traj_length_max=length;                        % update longest path length
       State.traj_length_min=length;                        % update shortest path length
   end
   if length>State.traj_length_max
       State.traj_length_max=length;                        % update longest path length
   end
   if length<State.traj_length_min
       State.traj_length_min=length;                        % update shortest path length
   end

   %% The expected path length should be within the interval of two basic path lengths
   % Find the path that is shorter than and closest to the expected path length (Call is as "bottom" path)
   if length<State.ideal_length
       if State.traj_index_bottom==0
           State.traj_index_bottom=i;
       elseif length>State.traj_length...
               (State.traj_index_bottom)
           State.traj_index_bottom=i;
       end
   end
   % Find the path that is longer than and closest to the expected path length (Call is as "top" path)
   if length>State.ideal_length
       if State.traj_index_top==0
           State.traj_index_top=i;
       elseif length<State.traj_length...
               (State.traj_index_top)
           State.traj_index_top=i;
       end
   end
end

%% Generate cooperative path
% In the absence of a 'bottom' path, directly output the 'top' path
if State.traj_index_bottom==0
    State.TrajSeq_Coop=State.TrajSeqCell{State.traj_index_top};
    State.optim_length=Traj_Length(State.TrajSeq_Coop);
    return;
end

TrajSeq=State.TrajSeqCell{State.traj_index_bottom};
[m,~]=size(TrajSeq);
invasion_bottom=0;
for i=1:m
    if TrajSeq(i,32)==1
       invasion_bottom=1;
    end
end

if invasion_bottom==1
    TrajSeq_new=TrajSeq;
    flag=1;
else
    % Use particle swarm optimization algorithm to adjust the radius of the starting and ending arcs of each path segments, 
    % so that the path length is as close as possible to the expected path length
    [TrajSeq_new,flag]=Traj_PSO(TrajSeq,State,ObsInfo,Property);
end

if flag==0
    TrajSeq_new=TrajSeq;
end

length_bottom=Traj_Length(TrajSeq_new);                     % Caluculate the "bottom" path length

if State.traj_index_top~=0                                  % In the presence of a 'top' path
    length_top=Traj_Length...                               % Caluculate the "top" path length
        (State.TrajSeqCell{State.traj_index_top});
    % Compare the "bottom" path and the "top" path to see which one has the closest length to the expected path length
    if abs(length_bottom-State.ideal_length) > abs(length_top-State.ideal_length)
        % Store the path with the closest length in TrajSeq_Cop
        State.TrajSeq_Coop=State.TrajSeqCell{State.traj_index_top};
    else                                                    
        State.TrajSeq_Coop=TrajSeq_new;                     
    end
% There is no 'top' path, and the expected path length is greater than all path lengths
else
    % No need to compare, simply set the optimized path as a cooperative path
    State.TrajSeq_Coop=TrajSeq_new;                        
end

State.optim_length=Traj_Length(State.TrajSeq_Coop);         % Calculate the cooperative path length
end

