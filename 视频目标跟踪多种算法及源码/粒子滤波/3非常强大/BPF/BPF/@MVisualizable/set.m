function obj = set(obj, varargin)
% SET           Set properties of the @MVisualizable object
%   obj = set(obj, 'propName', val, ...) sets the properties of the
%   @MVisualizable object. Possible 'propName' includes:
%       'width'             width of the figure
%       'height'            height of the figure
%       'title'             title of the figure
%
%   See also MVISUALIZABLE, GET

% Copyright 2006 Wei-Lwun Lu
% set.m version 1.0

propertyArgIn = varargin;
while length(propertyArgIn) >= 2,
    prop = propertyArgIn{1};
    val = propertyArgIn{2};
    propertyArgIn = propertyArgIn(3:end);
    switch prop
        case 'width'
            obj.width = val;
        case 'height'
            obj.height = val;
        case 'title'
            obj.title = val;
    end
end

