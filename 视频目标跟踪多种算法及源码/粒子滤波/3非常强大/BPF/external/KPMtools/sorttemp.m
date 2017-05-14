function [s,i] = sort(x,order)
% A stupid wrapper since Matlab < R13 doesn't let you sort in descending
% order

if nargin > 1
    o = order;
else
    o = 'ascending';
end

if strcmp('ascend',o)==1
    [s i] = sort(x);
elseif strcmp('descend',o)==1
    [s i] = sort(x);
    s = fliplr(s);
    i = fliplr(i);
else
    fprintf('error in sortemp\n');
end