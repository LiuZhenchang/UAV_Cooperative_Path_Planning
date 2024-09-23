%****************************************************************************************************************************
% Discription:  Use particle swarm optimization algorithm to adjust the radius of the starting and ending arcs of each path segments, 
%               so that the path length is as close as possible to the expected path length
% input:        TrajSeq                 Matrix of UAV path information
% input:        State                   Structure of flight paths information for the UAV
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
% output:       TrajSeq_new             Matrix of UAV path information
% output:       flag                    flag of path generation results
%****************************************************************************************************************************

function [TrajSeq_new,flag] = Traj_PSO(TrajSeq,State,ObsInfo,Property)
%% Define initialization information for PSO algorithm
[dubins_num,~]=size(TrajSeq);                               % Obtain the number of Dubins path segments
increment_num=dubins_num*2;                                 % Calculate the number of radius increments
number=100;                                                 % Set the number of particles
iter_max=20;                                                % Set the maximum number of iterations for PSO algorithm
demension=increment_num;                                    % Set the dimension of particle motion space
P_lim=zeros(demension,2);                                   % Initialize particle swarm position constraints
V_lim=zeros(demension,2);                                   % Initialize particle swarm velocity constraints
for i=1:demension                                           % Set particle swarm constraints
    P_lim(i,1)=0;
    P_lim(i,2)=Property.increment;
    V_lim(i,1)=-0.1*Property.radius;
    V_lim(i,1)=0.1*Property.radius;
end
c1=0.8;                                                     % Set inertial weight
c2=0.3;                                                     % Set self-learning factor
c3=0.3;                                                     % Set group learning factor

%% Initialize particle swarm state
position=zeros(number,demension);                           % Initialize the matrix of particle swarm position information
velocity=zeros(number,demension);                           % Initialize the matrix of particle swarm velocity information
pos_par_best=zeros(number,demension);                       % Initialize the optimal position for each particle
fit_par_best=zeros(number,1)+inf;                           % Initialize the optimal objective value for each particle
fit_gro_history=zeros(iter_max,1);                          % Initialize the minimum objective value for each iteration of PSO
for n=1:number
    for i=1:demension
        position(n,i)=...                                   % Randomly generate the position of particles
            P_lim(i,1)+(P_lim(i,2)-P_lim(i,1))*rand;
        velocity(n,i)=...                                   % Randomly generate the velocity of particles
            V_lim(i,1)+(V_lim(i,2)-V_lim(i,1))*rand;
    end
end

%% Update particle swarm state
iter=1;                                                     % Initialize iteration counter
while iter<=iter_max                                        % Iterative Calculation of PSO
    for n=1:number                                          % Calculate the information of each particle
        Increment=position(n,:);                            % Obtain increments
        [TrajSeq_new,flag]=Traj_Seq_Modification...         % Generate the modified path based on the primary path and radius increments
            (TrajSeq,Increment,ObsInfo,Property);
        %Plot_Traj_Single(TrajSeq_new,ObsInfo,Property,1)
        if flag~=0                                          % If the path exists
            length=Traj_Length(TrajSeq_new);                % Calculate path length
        end
        if flag==2                                          % If the path does not intersect with obstacles
            fitness=abs(length-State.ideal_length);         % Calculate the objective value of the current particle generated path
        else                                                % other conditons
            fitness=1e10;                                   % Set a larger objective value
        end
        if fitness<fit_par_best(n)                          % If the current objective value is less than the historical minimum value
            fit_par_best(n)=fitness;                        % Update the minimum objective value of the particle
            pos_par_best(n,:)=position(n,:);                % Update the minimum objective value position of the particle
        end
    end
    [fit_gro_best,index]=min(fit_par_best);                 % Update the minimum objective value of particle swarm
    pos_gro_best=pos_par_best(index,:);                     % Update the minimum objective value position of particle swarm
    fit_gro_history(iter)=fit_gro_best;                     % Record the minimum objective value for each iteration of PSO

    velocity=...                                            % Update particle swarm velocity direction
        c1*velocity+...                                     % Inertial term
        c2*rand*(pos_par_best-position)+...                 % Self learning term
        c3*rand*(repmat(pos_gro_best,number,1)-position);   % Group learning term
    for n=1:number
        for i=1:demension
            if velocity(n,i)>V_lim(i,2)                     % Limit the maximum veolocity value
                velocity(n,i)=V_lim(i,2);
            end
            if velocity(n,i)<V_lim(i,1)                     % Limit the minimum velocity value
                velocity(n,i)=V_lim(i,1);
            end
        end
    end

    position=position+velocity;                             % Update particle swarm position
    for n=1:number
        for i=1:demension
            if position(n,i)>P_lim(i,2)                     % Limit the maximum position value
                position(n,i)=P_lim(i,2);
            end
            if position(n,i)<P_lim(i,1)                     % Limit the minimum position value
                position(n,i)=P_lim(i,1);
            end
        end
    end
    iter=iter+1;                                            % Accululate counter 

end
%% Generate the optimized path
[~,index]=min(fit_par_best);                                % obtian the best particle
Increment=pos_par_best(index,:);                            % minimum objective value position of the particle (best increments)
[TrajSeq_new,flag]=Traj_Seq_Modification...                 % Generate the modified path corresponding to the best increments
    (TrajSeq,Increment,ObsInfo,Property);

end

