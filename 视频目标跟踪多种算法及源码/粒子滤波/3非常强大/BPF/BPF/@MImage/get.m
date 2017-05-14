function val = get(obj, propName)
% GET           Get properties of the @MImage object
%   get(obj) shows properties that are allowed to get.
%
%   val = get(obj, 'propName') get the properties of the @MImage object.
%   Possible 'propName' includes:
%       'height'        height of the image
%       'width'         width of the image
%       'image'         get the RGB image as a 3D array
%       'isColorImage'  whether the image is a color image
%       'hsvImage'      get the HSV image as a 3D array
%
%   See also MIMAGE, SET

% Copyright 2006 Wei-Lwun Lu
% get.m version 1.0

if nargin == 1
    strlist = {'height', 'width', 'image', 'isColorImage', 'hsvImage'};
    displayProperties(strlist);
else
    switch propName
        % image information
        case 'height'
            val = obj.height;
        case 'width'
            val = obj.width;
        case 'image'
            val = obj.image;
        case 'isColorImage'
            val = obj.isColorImage;
        case 'hsvImage'
            if obj.isColorImage
                val = rgb2hsv(obj.image);
            else
                val = obj.image;
            end

        otherwise
            error([propName,' Is not a valid @MImage property'])
    end
end
