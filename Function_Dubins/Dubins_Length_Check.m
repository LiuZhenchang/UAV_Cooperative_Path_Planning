%*********************************************************************************************************************
% Discription:  Detect whether UAV can generate path that meet the length requirements
% input:        length                  Expected path length
% input:        dubins_info             Basic Dubins path information
% output:       result                  Results of path can be generated 1 Yes, 0 No
% output:       dubins_best             The Dubins path closest to the expected length
%*********************************************************************************************************************

function [result,dubins_best] = Dubins_Length_Check(length,dubins_info)
result=0;
for type=1:4
    dubins_info=Dubins_Generate(dubins_info,type);              % Calculate complete Dubins path information
    %If the current trajectory is shorter than the expected trajectory, there is room for PSO optimization
    if dubins_info.traj.length<=length
        [fit_gro_history,dubins_best]=...                       % Adjust the radius of the starting and ending arcs of the current path 
            Dubins_PSO(length,dubins_info);                     % using particle swarm optimization algorithm
        if min(fit_gro_history)<0.2*length                      % If the minimum length error is within the allowable range
            result=1;                                           % Indicating that the UAV can generate a path that satisfies the length constraint
            return;                                             % Terminate the function, and the following types will no longer be calculated
        end        
    end
end
end

