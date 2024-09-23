%********************************************************************************************
% Discription:  Calculate path length
% input:        TrajSeqCell             Matrix of UAV path sequence
% output:       length                  Path length
%********************************************************************************************

function length = Traj_Length(TrajSeq)
[dubins_num,~]=size(TrajSeq);                                   % Obtain the number of segments in the path
length=0;                                                       % Initializa path length
for i=1:dubins_num                                              % Traverse all segments of the path
    length=length+TrajSeq(i,24);                                % Accumulate the length of each segment
end
end

