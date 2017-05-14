function obj = flipud(obj)
% FLIPUD        Flip the MImage from up to down
%   obj = flipud(obj) flips the MImage from up to down.
%
%   See also MIMAGE, FLIPLR, ROT90

% Copyright 2006 Wei-Lwun Lu
% flipud.m version 1.0

data = obj.image;

if ndims(data) == 2
    obj.image = flipud(data);
elseif ndims(data) == 3
    for k = 1 : size(data, 3)
        obj.image(:, :, k) = flipud(data(:, :, k));
    end
end
