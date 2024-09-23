%********************************************************************************************
% Discription:  Draw the path of the UAV
% input:        TrajSeq                 MAtrix of UAV path information
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
% input:        flag                    Options of plot alternative paths, 0: Not plot; 1: plot
%********************************************************************************************

function [o1,l1]=Plot_Traj_Single(TrajSeq,ObsInfo,Property,flag)
%% Initialize information 
scale=Property.scale;                                           % Set the drawing scale
[Traj_x,Traj_y]=Traj_Discrete(TrajSeq,Property);                % Obtain the discretized waypoint sequence
[~,n1]=size(Traj_x);                                            % Obtain the number of waypoints
[n2,~]=size(TrajSeq);                                           % Obtain the number of flight path segments
figure('name','UAV Trajectory');
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
l1=plot(Traj_x*scale,Traj_y*scale,'k');                         % Plot the path
l1.LineWidth=1.5;                                               % Set the path width
x_s=Traj_x(1);                                                  % Starting point x coordinate
y_s=Traj_y(1);                                                  % Starting point y coordinate
x_f=Traj_x(n1);                                                 % Ending point x coordinate
y_f=Traj_y(n1);                                                 % Ending point y coordinate
x_pt=[x_s,x_f];
y_pt=[y_s,y_f];
z_pt=[25000,25000];
pt=scatter3(x_pt*scale,y_pt*scale,z_pt*scale,80);               % Plot starting and ending points
pt.MarkerFaceColor='r';
pt.MarkerEdgeColor='k';

%% Determine whether to plot auxiliary lines based on the flag
if flag==1
    for i=1:n2
        theta=0:0.05:2*pi;
        xc_s=TrajSeq(i,7);                                      % Starting arc center x coordinate
        yc_s=TrajSeq(i,8);                                      % Starting arc center y coordinate
        R_s=TrajSeq(i,5);                                       % Starting arc radius
        xs_temp=xc_s+R_s*cos(theta);
        ys_temp=yc_s+R_s*sin(theta);
        l2=plot(xs_temp*scale,ys_temp*scale,':b');              % Plot starting arc
        l2.LineWidth=0.5;

%         if TrajSeq(i,23)~=0
%             xc_f=TrajSeq(i,18);                               % Ending arc center x coordinate
%             yc_f=TrajSeq(i,19);                               % Ending arc center y coordinate
%             R_f=TrajSeq(i,16);                                % Ending arc radius
%             xf_temp=xc_f+R_f*cos(theta);
%             yf_temp=yc_f+R_f*sin(theta);
%             l3=plot(xf_temp*scale,yf_temp*scale,':b');        % Plot ending arc
%             l3.LineWidth=0.5;
%         end
    end
end

%% If the UAV invades the obatacle (threat circle), draw the compressed threat circle
flag_invasion=0;
for i=1:n2
    if TrajSeq(i,25)==0
        continue;
    end
    if TrajSeq(i,32)==1&&...                                    % If the UAV invades the obstacle
            TrajSeq(i,16)<ObsInfo(TrajSeq(i,25),3)              % And the radius of the ending arc is smaller than the radius of the obstacle
        xc_f=TrajSeq(i,18);                                     % Obtain ending arc center x coordinate
        yc_f=TrajSeq(i,19);                                     % Obtain ending arc center y coordinate
        R_f=TrajSeq(i,16);                                      % Obtain ending arc radius
        theta=0:0.05:2*pi;
        xf_temp=xc_f+R_f*cos(theta);
        yf_temp=yc_f+R_f*sin(theta);
        l4=plot(xf_temp*scale,yf_temp*scale,'m');              % Plot ending arc
        l4.LineWidth=1;
        flag_invasion=1;
    end

end
%% Set figure parameters
set(gcf,'unit','inches','position',[0,0,6,4.5]);
set(gca,'FontName','Times New Roman','FontSize',12);
%set(gca,'Position',[0.06,0.15,0.9,0.7]);
xlabel('$X/m$','Interpreter','latex');
ylabel('$Y/m$','Interpreter','latex');
zlabel('$Y/m$','Interpreter','latex');
xlim([-200,600]);
ylim([-300,350]);
grid on;
box on;
if flag_invasion==1&&flag==1
    L=legend([l1,o1,l4],{'Path',...
        'Threaten Area','Compressed Area'});
    L.Location='northeast';
end
%axis equal;
end

