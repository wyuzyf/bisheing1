function obj = rot90(obj, k)
% ROT90         Rotate the @MImage counterclockwise 90 degree
%   obj = rot90(obj) rotates the MImage counterclockwise 90 degree.
%
%   obj = rot90(obj, k) rotates the MImage counterclockwise k*90 degree.
%
%   See also MIMAGE, FLIPLR, FLIPUD

% Copyright 2006 Wei-Lwun Lu
% rot90.m version 1.0

data = obj.image;

if ndims(data) == 2
    if nargin == 1
        obj.image = rot90(data);
    else
        obj.image = rot90(data, k);
    end
elseif ndims(data) == 3
    obj.image = [];
    
    for layer = 1 : size(data, 3)
        if nargin == 1
            obj.image(:, :, layer) = rot90(data(:, :, layer));
        else
            obj.image(:, :, layer) = rot90(data(:, :, layer), k);
        end
    end
end

obj.height = size(obj.image, 1);
obj.width  = size(obj.image, 2);
