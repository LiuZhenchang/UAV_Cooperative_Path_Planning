%********************************************************************************************
% Discription:  Draw the alternative paths and cooperative paths of all UAVs
% input:        Coop_State              Cell array of UAVs paths information
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
% input:        flag                    Options of plot alternative paths, 0: Not plot; 1: plot
% input:        demo                    demo index
%********************************************************************************************

function Plot_Traj_Coop(Coop_State,ObsInfo,Property,flag,demo)
%% Initialize information 
[~,n]=size(Coop_State);
scale=Property.scale;                                               % Set the drawing scale
px=zeros(1,2*n);
py=zeros(1,2*n);
figure;
hold on;

%% Plot obstacles
theta=0:0.05:2*pi;
[obs_num,~]=size(ObsInfo);
for i=1:obs_num
    xo_temp=ObsInfo(i,1)+ObsInfo(i,3)*cos(theta);
    yo_temp=ObsInfo(i,2)+ObsInfo(i,3)*sin(theta);
    o1=plot(xo_temp*scale,yo_temp*scale,'r');
    o1.LineWidth=1.5;
    s=sprintf('%d',i);
    text(ObsInfo(i,1)*scale,ObsInfo(i,2)*scale,s);
end

%% Plot the path and its starting and ending points
for i=1:n
    if flag==1                                                      % Plot alternative paths
        [~,m]=size(Coop_State(i).TrajSeqCell);                      % Obtain the path number in TrajSeqCell
        for j=1:m                                                   % Traverse each path
            [Traj_x,Traj_y]=Traj_Discrete...                        % Obtain a discretized waypoint sequence
                (Coop_State(i).TrajSeqCell{j},Property);
            hold on;
            l2=plot(Traj_x*scale,Traj_y*scale,'b');                 % Plot alternative path
            l2.LineWidth=1;                                         % Set the path width
            %l2.Color=[1 1 1]*0.7;                                  % Grey opaque mode, transparency does not overlap
            l2.Color(4)=0.3;                                        % Gray transparent mode, transparency will be overlaid
        end
    end
    [~,c]=size(Traj_x);                                             % Obtain the number of discrete waypoints
    px(i)=Traj_x(1);                                                % Obtain the starting point x coordinate
    py(i)=Traj_y(1);                                                % Obtain the starting point y coordinate
    px(n+i)=Traj_x(c);                                              % Obtain the ending point x coordinate
    py(n+i)=Traj_y(c);                                              % Obtain the ending point y coordinate
end
pt=scatter(px*scale,py*scale,80);                                   % Plot starting and ending points
pt.MarkerFaceColor='r';
pt.MarkerEdgeColor='k';

for i=1:n                                                           % Plot cooperative path of each UAV
    [Traj_x,Traj_y]=Traj_Discrete...
        (Coop_State(i).TrajSeq_Coop,Property);                      % Obtain the discrete waypoints sequence
    hold on;
    l1=plot(Traj_x*scale,Traj_y*scale,'k');                         % Plot cooperative path
    l1.LineWidth=1.5;                                               % Set the path width
end

%% Set figure parameters
switch demo
    case 1
        set(gcf,'unit','inches','position',[0,0,6,4.5]);
        xlim([-150,600]); 
        ylim([-250,350]);
    case 2
        set(gcf,'unit','inches','position',[0,0,12,4]);
        xlim([-50,1050]);
        ylim([-100,300]);
end

set(gca,'FontName','Times New Roman','FontSize',12);
xlabel('$X/m$','Interpreter','latex');
ylabel('$Y/m$','Interpreter','latex');
zlabel('$Y/m$','Interpreter','latex');
grid on;
box on;
L=legend([l1,l2,o1],{'Path-Cooperative',...
    'Path-Alternative','Threaten Area'});
L.Location='northeast';
L.FontSize=12;

end

