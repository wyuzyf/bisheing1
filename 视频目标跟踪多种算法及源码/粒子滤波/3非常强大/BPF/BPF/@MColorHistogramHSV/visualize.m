function visualize(obj)
% VISUALIZE     Visualize the @MColorHistogramHSV object
%   visualize(obj) visualizes the @MColorHistogramHSV object.
%
%   Example:
%     obj = MColorHistogramHSV('moon.tif');
%     visualize(obj);
%
%   See also MCOLORHISTOGRAMHSV, MVISUALIZABLE

% Copyright 2006 Wei-Lwun Lu
% visualize.m version 1.0

% visualize the MImage
visualize(obj.MImage);

% visualize the HSV color histogram
[hshist, vhist] = gethistogram(obj, [], 1);

figure;
bar3(reshape(hshist, [obj.hbins, obj.sbins]));
title('H-S Channels');

figure;
bar(vhist);
title('V Channel');
