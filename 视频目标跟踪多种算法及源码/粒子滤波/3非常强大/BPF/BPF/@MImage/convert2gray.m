function obj = convert2gray(obj)
% CONVERT2GRAY  Convert the color image to a gray image
%   obj = convert2gray(obj) converts the color image <obj) to a gray image.
%
%   Example:
%
%   See also MIMAGE, GET, SET

% Copyright 2006 Wei-Lwun Lu
% convert2gray.m version 1.0

img = get(obj, 'image');
img = rgb2gray(img);

obj = MImage(img);
