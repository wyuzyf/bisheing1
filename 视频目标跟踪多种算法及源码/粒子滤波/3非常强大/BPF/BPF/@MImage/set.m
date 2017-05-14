function obj = set(obj, varargin)
% SET           Set properties of the @MImage object
%   set(obj)
%
%   obj = set(obj, 'propName', val, ...) sets the properties of the @MImage 
%   object. Possible 'propName' includes:
%     'image'   the image data (2D / 3D matrix)
%
%   See also MIMAGE, GET

% Copyright 2006 Wei-Lwun Lu
% set.m version 1.0

if nargin == 1
    strlist = {'image'};
    displayProperties(strlist);
else
    propertyArgIn = varargin;
    while length(propertyArgIn) >= 2,
        prop = propertyArgIn{1};
        val = propertyArgIn{2};
        propertyArgIn = propertyArgIn(3:end);
        switch prop
            % image info
            case 'image'
                obj = setimage(obj, val);

        end
    end
end


function obj = setimage(obj, val)

if ndims(val) == 2
    % if gray image
    obj.isColorImage = 0;
    obj.image        = val;
    obj.height       = size(val, 1);
    obj.width        = size(val, 2);
elseif ndims(val) == 3
    % if gray image
    obj.isColorImage = 1;
    obj.image        = val;
    obj.height       = size(val, 1);
    obj.width        = size(val, 2);
else
    error('Invalid input image.');
end

obj.image    = im2double(obj.image);


