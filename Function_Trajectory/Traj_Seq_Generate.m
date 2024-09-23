%****************************************************************************************************************************
% Discription:  Generate complete path sequences based on the results of each path planning step
% input:        TrajInfoCell            Database to store path segments infomation (path segment information storage cell)
% output:       TrajSeqCell             Cell Array of avaliable paths
%****************************************************************************************************************************

function TrajSeqCell = Traj_Seq_Generate(TrajInfoCell)
%% Initialize information
[~,n1]=size(TrajInfoCell);                                              % Obtain the length of path segment information storage cell
c1=0;                                                                   % Initialize counter 1 and count the number of path planning steps
c4=0;                                                                   % Initialize counter 4 and count the number of path segments that have reached the endpoint
c5=0;                                                                   % Initialize counter 5 and count the number of path reaching the endpoint
[~,info_length]=size(TrajInfoCell{1,1});                                % Obtain the length of information for a path segment

%% Count the number of path planning steps and path segments reaching the endpoint
for i=1:n1                                                              % Traverse each path segment information storage cell
    if TrajInfoCell{1,i}(1,1)==0                                        % If the matrix (1,1) in the cell is 0
        break;                                                          % Indicates that the cell has no stored data, terminate the loop
    end
    c1=c1+1;                                                            % Count the number of cells storing data (number of steps in path planning)
end
for k=1:c1
    TrajInfoFinal=TrajInfoCell{1,k};                                    % Obtain the path segment information matrix planned in the final step
    [n2,~]=size(TrajInfoFinal);                                         % Obtain the number of rows in the path segment information matrix
    for i=1:n2                                                          % Traverse every row in the path segment information matrix
        if TrajInfoFinal(i,1)==0                                        % If the first element of the path segment information vector is 0
            break;                                                      % Indicates that the row vector does not store path segment information, terminate the loop

        end
        if TrajInfoFinal(i,33)~=0                                       % If the path segment reaches the endpoint
            c4=c4+1;                                                    % Update the number of path segments reaching the endpoint
        end
    end
end
TrajSeqCell=cell(1,c4);                                                 % Initialize path sequence cells based on the c4

% for i=1:c4                                                            % Initialize a zero matrix in each cell
%     TrajSeqCell{1,i}=...                                              % The number of rows in the matrix is the number of steps
%         zeros(k,info_length);                                         % The number of columns in the matrix is the length of the path segment information
% end

%% Traverse the corresponding path segments for each step
for k=1:c1                                                              % Traverse the path segment planning results for each step
    c2=0;                                                               % Initialize counter 2 to count the number of path segments under the current steps
    c3=0;                                                               % Initialize counter 3 to count the number of path segments reaching the endpoint under the current steps
    TrajInfoFinal=TrajInfoCell{1,k};                                    % Obtain the path segment information matrix planned in the final step
    [n2,~]=size(TrajInfoFinal);                                         % Obtain the number of rows in the path segment information matrix

    %% Count the number of path segments (c2) and path segments reaching the endpoint (c3) under the current steps
    for i=1:n2                                                          % Traverse every row in the path segment information matrix
        if TrajInfoFinal(i,1)==0                                        % If the first element of the path segment information vector is 0
            break;                                                      % Indicates that the row vector does not store path segment information, terminate the loop
        end
        if TrajInfoFinal(i,33)~=0                                       % If the path segment reaches the endpoint
            c3=c3+1;                                                    % Update the number of path segments reaching the endpoint
        end
        c2=c2+1;                                                        % count the number of path segments under the current steps
    end
    if c3==0                                                            % If no path segment reaches the endpoint
        continue;                                                       % Skip the current loop
    end
    TrajSeqCell_Temp=cell(1,c3);                                        % Initialize the path sequence cells based on c3
    for i=1:c3                                                          % Initialize a zero matrix in each cell
        TrajSeqCell_Temp{1,i}=...                                       % he number of rows in the matrix is the number of steps
            zeros(k,info_length);                                       % The number of columns in the matrix is the length of the path segment information
    end

    %% Generate matrices for each path sequence
    c3=0;
    for i=1:c2                                                          % Traverse each path segment planned in the last step
        if TrajInfoFinal(i,33)~=0                                       % If the path segment reaches the endpoint
            c3=c3+1;                                                    % Update the number of path segments reaching the endpoint
        else
            continue;
        end
        TrajSeqTemp=zeros(k,info_length);                               % Initialize temporary path sequence matrix
        TrajInfoEnd=TrajInfoFinal(i,:);                                 % Obtain the last segment of the path sequence
        TrajSeqTemp(1,:)=TrajInfoEnd;                                   % Store the last segment of path in the temporary path sequence matrix
        %% Reverse tracing generates path sequences
        for j=2:k                                                       % Traverse each path segment information matrix in TrajInfoCell in reverse order
            TrajInfoTemp=TrajInfoCell{1,k-j+1};                         % Obtain the current path segment information matrix
            [n2,~]=size(TrajInfoTemp);                                  % Obtain the number of rows in the path segment information matrix
            for m=1:n2                                                  % Traverse every row in the path segment information matrix
                if TrajInfoTemp(m,1)==0                                 % If the first element of the path segment information vector is 0
                    break;                                              % terminate the loop
                end
                if TrajInfoEnd(1,2)==TrajInfoTemp(m,13)&&...            % If the starting point of the current path segment is the same as the 
                        TrajInfoEnd(1,3)==TrajInfoTemp(m,14)&&...       % ending point of the previous path segment,
                        TrajInfoEnd(1,4)==TrajInfoTemp(m,15)            % it means that the two path segments are continuous
                    TrajInfoEnd=TrajInfoTemp(m,:);                      % Update the current path segment information to the previous path segment information
                    TrajSeqTemp(j,:)=TrajInfoEnd;                       % Store the current path segment information in the temporary path sequence matrix
                end
            end
        end
        %% Forward storage of path segment sequence
        for j=1:k
            TrajSeqCell_Temp{1,c3}(j,:)=TrajSeqTemp(k-j+1,:);
        end
    end
    for i=1:c3
        c5=c5+1;
        TrajSeqCell{1,c5}=TrajSeqCell_Temp{1,i};
    end
end

end

