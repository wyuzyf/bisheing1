function obj = gaussblur(obj, width, sigma)
% GAUSSBLUR     Blur a MImage with a Gaussian filter
%   obj = gaussblur(obj) blurs the MImage with a Gaussian filter with width
%   3 and sigma 1.
%
%   obj = gaussblur(obj, width, sigma) blurs the MImage with a Gaussian
%   filter given the width and sigma of the Gaussian filter.
%
%   See also MIMAGE

% Copyright 2006 Wei-Lwun Lu
% gaussblur.m version 1.0

if nargin == 1
    width = 3;
    sigma = 1;
end

% create Gaussian filter
gaussFilter = fspecial('gaussian', width, sigma);

% blur the image using the Gaussian filter
imgin  = obj.image;
imgout = imfilter(imgin, gaussFilter, 'replicate', 'corr');

obj = MImage(imgout);
