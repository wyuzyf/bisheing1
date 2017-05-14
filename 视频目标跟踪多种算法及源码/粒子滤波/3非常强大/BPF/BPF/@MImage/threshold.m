function obj = threshold(obj, thresh, type)
% THRESHOLD     Threshold the image
%   obj = threshold(obj, thresh, 'down') forces the pixels whose value
%   below <thresh> to be zero.
%
%   Example:
%       obj = MImage('peppers.png');
%       visualize(obj);
%       visualize(threshold(obj, 0.5));
%
%   See also MIMAGE

% Copyright 2006 Wei-Lwun Lu
% threshold.m version 1.0

% process arguments
if nargin == 2
    type = 'down';
end

% threshold the image
switch type
    case 'down'
        obj.image(obj.image < thresh) = 0;
    otherwise
        error('Unknown type.');
end
