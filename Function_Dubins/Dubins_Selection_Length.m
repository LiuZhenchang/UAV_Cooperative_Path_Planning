%********************************************************************************************
% Discription:  Select the shortest path
% input:            TrajCollect             MAtrix of UAV paths information
% input:            i                       Row number of TrajCollect
% in/output:        length_min              Minimum length
% in/output:        index                   Selected path index (number)
%********************************************************************************************

function [length_min,index] = Dubins_Selection_Length(TrajCollect,length_min,index,i)
    length=TrajCollect(i,24);

    if length==0
        return;
    end

    if length_min==0
        length_min=length;
        index=i;
    end

    if length_min>length
        length_min=length;
        index=i;
    end
end

