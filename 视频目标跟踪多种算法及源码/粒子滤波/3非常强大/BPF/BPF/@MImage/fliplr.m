function obj = fliplr(obj)
% FLIPLR        Flip the MImage from left to right
%   obj = fliplr(obj) flips the MImage from left to right.
%
%   See also MIMAGE, FLIPUD, ROT90

% Copyright 2006 Wei-Lwun Lu
% fliplr.m version 1.0

data = obj.image;

if ndims(data) == 2
    obj.image = fliplr(data);
elseif ndims(data) == 3
    for k = 1 : size(data, 3)
        obj.image(:, :, k) = fliplr(data(:, :, k));
    end
end
