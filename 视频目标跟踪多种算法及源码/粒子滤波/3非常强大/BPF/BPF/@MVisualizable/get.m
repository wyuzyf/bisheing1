function val = get(obj, propName)
% GET           Get properties of the @MVisualizable object
%   val = get(obj, 'propName') get the properties of the @MVisualizable 
%   object. Possible 'propName' includes:
%       'width'         width of the figure
%       'height'        height of the figure
%       'title'         title of the figure
%
%   See also MVISUALIZABLE, SET

% Copyright 2006 Wei-Lwun Lu
% get.m version 1.0

switch propName
    case 'width'
        val = obj.width;
    case 'height'
        val = obj.height;
    case 'title'
        var = obj.title;
    otherwise
        error([propName,' Is not a valid @MVisualizable property'])
end