function val = get(obj, propName)
% GET           Get properties of the @MIntegralImage object
%   val = get(obj, 'propName') get the properties of the @MIntegralImage
%   object. Possible 'propName' includes:
%       'width'     width of the image
%       'height'    height of the image
%
%   Example:
%       obj = MIntegralImage(pascal(10));
%       width = get(obj, 'width');
%
%   See also MINTEGRALIMAGE, SET

% Copyright 2006 Wei-Lwun Lu
% get.m version 1.0

switch propName
    case 'width'
        val = obj.width;
    case 'height'
        val = obj.height;
    case 'integralImage'
        val = obj.integralImage;
        
    otherwise
        error([propName,' Is not a valid @MIntegralImage property'])
end