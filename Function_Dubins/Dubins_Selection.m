%********************************************************************************************
% Discription:  Selection paths in TrajCollect with specific condition 
% input:        TrajCollect             MAtrix of UAV paths information
% input:        ObsInfo                 Matrix of obstacles information
% input:        Property                Structure of path planning parameters
% input:        stage                   Path planning stage
% output:       index                   Selected path index (number)
% output:       ObsCur                  Number of the first obstacle on each path
%********************************************************************************************

function [index,ObsCur] = Dubins_Selection(TrajCollect,ObsInfo,Property,stage)

[n,~]=size(TrajCollect);
[m,~]=size(ObsInfo);
if stage==1
    flag=Property.selection1;
else
    flag=Property.selection2;
end
switch flag
    %  Select paths that not intersect with obstacles
    case 1
        IndexTemp=zeros(1,Property.max_info_num);
        count=0;
        for i=1:n
            length=TrajCollect(i,24);
            if length==0
                continue;
            end
            obs_num=TrajCollect(i,26);
            obs_index(1:5)=TrajCollect(i,27:31);
            if obs_num==0&&obs_index(1)==0
                count=count+1;
                IndexTemp(1,count)=i;
            end
            %Plot_Traj_Single(TrajCollect(i,:),ObsInfo,Property)
        end
        if count==0
            index=0;
        else
            index=zeros(1,count);
            index(1,:)=IndexTemp(1,1:count);
        end

    % Select paths which turning angle does not exceed 3 * pi/2
    case 2
        IndexTemp=zeros(1,Property.max_info_num);
        count=0;
        for i=1:n
            length=TrajCollect(i,24);
            if length==0
                continue;
            end
            psi_s=TrajCollect(i,12);
            psi_f=TrajCollect(i,23);
            if abs(psi_s)+abs(psi_f)<3*pi/2
                count=count+1;
                IndexTemp(1,count)=i;
            end
        end
        if count==0
            index=0;
        else
            index=zeros(1,count);
            index(1,:)=IndexTemp(1,1:count);
        end
    % Select paths which simultaneously satisfying cases 1 and 2
    case 3
        IndexTemp=zeros(1,Property.max_info_num);
        count=0;
        for i=1:n
            length=TrajCollect(i,24);
            if length==0
                continue;
            end
            obs_num=TrajCollect(i,26);
            obs_index(1:5)=TrajCollect(i,27:31);
            psi_s=TrajCollect(i,12);
            psi_f=TrajCollect(i,23);
            if obs_num==0&&obs_index(1)==0
                if abs(psi_s)+abs(psi_f)<3*pi/2
                    count=count+1;
                    IndexTemp(1,count)=i;
                end
            end
            %Plot_Traj_Single(TrajCollect(i,:),ObsInfo,Property)
        end
        if count==0
            index=0;
        else
            index=zeros(1,count);
            index(1,:)=IndexTemp(1,1:count);
        end
    % Select the obstacle free and shortest path
    case 4
        index=0;
        length_min=0;
        for i=1:n
            obs_num=TrajCollect(i,26);
            obs_index(1:5)=TrajCollect(i,27:31);
            if obs_num==0&&obs_index(1)==0
                [length_min,index]=Dubins_Selection_Length...
                    (TrajCollect,length_min,index,i);
            end
        end
    % Select the shortest path that meets the turning angle constraint
    case 5
        index=0;
        length_min=0;
        for i=1:n
            psi_s=TrajCollect(i,12);
            psi_f=TrajCollect(i,23);
            if abs(psi_s)+abs(psi_f)<3*pi/2
                [length_min,index]=Dubins_Selection_Length...
                    (TrajCollect,length_min,index,i);
            end
        end
    % Select the shortest path
    case 6
        index=0;
        length_min=0;
        for i=1:n
            [length_min,index]=Dubins_Selection_Length...
                (TrajCollect,length_min,index,i);
        end
    % Select the path that meets the turning angle constraint and poses the least threat
    case 7
        index=0;
        length_min=0;
        threat_min=0;
        for i=1:n
            psi_s=TrajCollect(i,12);
            psi_f=TrajCollect(i,23);
            if abs(psi_s)+abs(psi_f)<3*pi/2
                [threat_min,length_min,index]=Dubins_Selection_Threat...
                    (TrajCollect,ObsInfo,threat_min,length_min,index,i);
            end
        end
    % Select the path with the least threat
    case 8
        index=0;
        length_min=0;
        threat_min=0;
        for i=1:n
            [threat_min,length_min,index]=Dubins_Selection_Threat...
                (TrajCollect,ObsInfo,threat_min,length_min,index,i);
        end

end

%% Obtain the number of the first obstacle on the path
obs_count=0;
ObsCur=zeros(1,m);
for i=1:n
    length=TrajCollect(i,24);
    if length~=0
        obs_index(1:5)=TrajCollect(i,27:31);
        obs_flag=1;
        for j=1:obs_count+1
            if ObsCur(j)== obs_index(1)||...
                    Property.obs_last==obs_index(1)
                obs_flag=0;
            end
        end
        if obs_flag==1
            obs_count=obs_count+1;
            ObsCur(1,obs_count)=obs_index(1);
        end
    end
end
end



