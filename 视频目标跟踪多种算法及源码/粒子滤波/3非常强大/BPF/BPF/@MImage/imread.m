function obj = imread(obj, filename)
% IMREAD        Load the @MImage object from a image file
%   obj = imread(obj, filename) loads the @MImage object given the filename
%   of an image file.
%
%   See also MIMAGE

% Copyright 2006 Wei-Lwun Lu
% imread.m version 1.0

info = imfinfo(filename);

switch info(1).ColorType
    case 'truecolor'
        X = imread(filename);
        obj.image        = X;
        obj.height       = size(X, 1);
        obj.width        = size(X, 2);
        obj.isColorImage = 1;
    case 'grayscale'
        X = imread(filename);
        obj.image        = X;
        obj.height       = size(X, 1);
        obj.width        = size(X, 2);
        obj.isColorImage = 0;
    case 'indexed'
        [X, map] = imread(filename);
        rgb      = ind2rgb(X, map);
        
        obj.image        = rgb;
        obj.height       = size(rgb, 1);
        obj.width        = size(rgb, 2);
        obj.isColorImage = 1;
    otherwise
        error('Invalid ColorType.');
end

obj.image    = im2double(obj.image);

