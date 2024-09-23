%****************************************************************************************************************************
% Discription:  Discretize path sequence into waypoints sequence
% input:        TrajSeq                 Matrix of primary path sequence
% input:        Property                Structure of path planning parameters
% output:       Traj_x                  Array of waypoints x coordinate
% output:       Traj_y                  Array of waypoints y coordinate
%****************************************************************************************************************************

function [Traj_x,Traj_y] = Traj_Discrete(TrajSeq,Property)
[r,~]=size(TrajSeq);                                            % Obtain the number of Dubins path segments
start_info=zeros(1,4);                                          % Initialize starting info
finish_info=zeros(1,4);                                         % Initialize ending info
ns=Property.ns;                                                 % Obtain number of discrete points in the starting arc
nl=Property.nl;                                                 % Obtain number of discrete points in the straight line
nf=Property.nf;                                                 % Obtain number of discrete points in the ending arc
x_temp=zeros(1,10000);                                          % Initialize waypoints x coordinate sequence
y_temp=zeros(1,10000);                                          % Initialize waypoints y coordinate sequence
count=1;                                                        % Initialize waypoints conunter
for i=1:r                                                       % Traverse every path segment in sequence
    type=TrajSeq(i,1);                                          % Obtain current path segment type
    if type==0                                                  % If the path type is 0
        break;                                                  % terminate loop
    end    
    start_info(1)=TrajSeq(i,2);                                 % Starting point x coordinate
    start_info(2)=TrajSeq(i,3);                                 % Starting point y coordinate
    start_info(3)=TrajSeq(i,4);                                 % Starting heading angle
    start_info(4)=TrajSeq(i,5);                                 % Starting arc radius
    finish_info(1)=TrajSeq(i,13);                               % Ending point x coordinate
    finish_info(2)=TrajSeq(i,14);                               % Ending point y coordinate
    finish_info(3)=TrajSeq(i,15);                               % Ending heading angle
    finish_info(4)=TrajSeq(i,16);                               % Ending arc radius

    dubins_info=Dubins_Init(start_info,finish_info);            % Initialize basic Dubins path structure
    dubins_info=Dubins_Generate(dubins_info,type);              % Generate complete path information based on basic path information and path type
    [dubins_x,dubins_y]=Dubins_Discret(dubins_info,ns,nl,nf);   % Discretize Dubins path into waypoints sequence

    [~,n]=size(dubins_x);                                       % Obtain the number of waypoints for the current Dubins path segment
    for j=1:n                                                   % Sequentially store the waypoints of the current path segment
        x_temp(count)=dubins_x(j);                              
        y_temp(count)=dubins_y(j);                              
        count=count+1;                                          % Update waypoint counter
    end
end
Traj_x=zeros(1,count-1);
Traj_y=zeros(1,count-1);
Traj_x(:)=x_temp(1:count-1);                                    % Output waypoints x coordinate
Traj_y(:)=y_temp(1:count-1);                                    % Output waypoints y coordinate
end

