%****************************************************************************************************************************
% Discription:  Use particle swarm optimization algorithm to adjust the radius of the starting and ending arcs of the path, 
%               so that the path length is as close as possible to the expected path length
% input:        length                  Expected path length
% input:        dubins_info             Basic Dubins path information
% output:       fit_gro_history         Minimum objective value for each iteration of PSO
% output:       dubins_temp             Optimized Dubins path information
%****************************************************************************************************************************

function [fit_gro_history,dubins_temp] = Dubins_PSO(length,dubins_info)
if dubins_info.traj.length>length
    error('No avaliable solution\n');
end
%% Define initialization information for PSO algorithm
number = 30;                                                % Set the number of particles
demension = 2;                                              % Set the dimension of particle motion space
iter_max = 20;                                              % Set the maximum number of iterations for PSO algorithm
R_s=dubins_info.start.R;                                    % Obtain the minimum radius of starting arc
R_f=dubins_info.finish.R;                                   % Obtain the minimum radius of ending arc
type=dubins_info.traj.type;                                 % Obtain Dubins path type
P_lim=[R_s,R_s*5;R_f,R_f*5];                                % Define particle swarm position constraints
V_lim=0.1*[-R_s,R_s;-R_f,R_f];                              % Define particle swarm velocity constraints
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
    dubins_temp=dubins_info;                                % Generate temporary Dubins structures for iteration
    for n=1:number                                          % Calculate the information of each particle
        dubins_temp.start.R=position(n,1);                  % The x-coordinate of the particle is the radius of the starting arc
        dubins_temp.finish.R=position(n,2);                 % The y-coordinate of the particle is the radius of the starting arc
        dubins_temp=Dubins_Generate(dubins_temp,type);      % Generate the dubins path corresponding to the particle
        fitness=abs(length-dubins_temp.traj.length);        % Calculate the objective value of the particle
        if fitness<fit_par_best(n)                          % If the current objective value is less than the historical minimum value
            fit_par_best(n)=fitness;                        % Update the minimum objective value of the particle
            pos_par_best(n,:)=position(n,:);                % Update the minimum objective value position of the particle
        end
    end
    [fit_gro_best,index]=min(fit_par_best);                 % Update the minimum objective value of particle swarm
    pos_gro_best=pos_par_best(index,:);                     % Update the minimum objective value position of particle swarm
    fit_gro_history(iter)=fit_gro_best;                     % Record the minimum objective value for each iteration of PSO
    
    %      if iter>1
    %          if abs(fit_gro_best-fit_gro_history(iter-1))<1
    %              return;
    %          end
    %      end
    
    dubins_temp.start.R=pos_par_best(index,1);              % The x-coordinate of the particle is the radius of the starting arc
    dubins_temp.finish.R=pos_par_best(index,2);             % The y-coordinate of the particle is the radius of the starting arc
    dubins_temp=Dubins_Generate(dubins_temp,type);          % Generate the dubins path corresponding to the particle
    
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
end

