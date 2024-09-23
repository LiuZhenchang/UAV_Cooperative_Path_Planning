%********************************************************************************************
% Discription:  Draw all paths connecting the starting and ending points, as well as their modified paths
% input:        TrajSeqCell             Cell array of UAV paths information
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
%********************************************************************************************

function Plot_Traj_Multi_Modification(TrajSeqCell,ObsInfo,Property)
[~,n]=size(TrajSeqCell);                                            % Obtain paths number
scale=Property.scale;                                               % Obtain the drawing scale
for i=1:n
    TrajSeq=TrajSeqCell{1,i};
    [dubins_num,~]=size(TrajSeq);                                   % Obtain the number of Dubins paths
    increment_num=dubins_num*2;                                     % Calculate the number of increments 
                                                                    % (increments of starting circle radius and ending circle radius)
    Increment=zeros(1,increment_num);                               % Initializ the array of radius increments
    if i==1
        [o1,l1]=Plot_Traj_Single(TrajSeq,ObsInfo,Property,0);       % Plot basic image and standard paths
    else
        [Traj_x,Traj_y]=Traj_Discrete(TrajSeq,Property);            % Obtain the discrete waypoints sequence
        hold on;
        l1=plot(Traj_x*scale,Traj_y*scale,'k');                     % Plot primary paths
        l1.LineWidth=1.5;
    end
    
    for j=1:100                                                     % Generate modified paths randomly through a loop
        for k=1:increment_num
            Increment(k)=rand*Property.increment;                   % Randomly generate increments greater than 0
        end
        [TrajSeq_new,flag]=Traj_Seq_Modification...                 % Generate a new path sequence matrix based on increments
            (TrajSeq,Increment,ObsInfo,Property);
        [dubins_num,~]=size(TrajSeq_new);                           % Obtain the number of Dubins path segments
        if flag==2&&TrajSeq_new(dubins_num,23)<6                    % If the path does not intersect with obstacles and the ending arc is less than 6rad
            hold on;                                                % Draw modified paths on the same figure
            [Traj_x,Traj_y]=Traj_Discrete(TrajSeq_new,Property);    % Obtain the discrete waypoints sequence
            l2=plot(Traj_x*scale,Traj_y*scale,'k');                 % Plot modified paths
            l2.LineWidth=0.5;                                       % Set the path width
            l2.Color(4)=0.2;                                        % Set the path transparency
        end
    end
end
L=legend([l1,l2,o1],{'Path-Basic',...
    'Path-Modification','Threaten Area'});
L.Location='northeast';
L.FontSize=12;
end

