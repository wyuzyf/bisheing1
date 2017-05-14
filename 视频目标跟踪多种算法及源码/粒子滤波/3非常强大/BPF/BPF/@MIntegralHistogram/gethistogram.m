function histogram = gethistogram(obj, rect, isNormalized)
% GETHISTOGRAM  Get the histogram of a given region
%   histogram = gethistogram(obj) gets the histogram of the entire image.
%
%   histogram = gethistogram(obj, rect) gets the histogram of a region 
%   defined in <rect>. <rect> = [x1, y1, x2, y2] where (x1, y1) is the 
%   inclusive upper-left corner of the region, and (x2, y2) is the 
%   inclusive lower-right corner of the region.
%
%   histogram = gethistogram(obj, rect, isNormalized) does the same thing
%   as histogram = gethistogram(obj, rect). Instead, if isNormalized = 1,
%   then the histogram will be normalized (sum to 1).
%
%   Example:
%     load(which('MIntegralHistogram/private/testMat.mat'));
%     obj = MIntegralHistogram(indexImage);
%     hist1 = gethistogram(obj);
%
%   See also MINTEGRALHISTOGRAM, GET, SET

% Copyright 2006 Wei-Lwun Lu
% gethistogram.m version 1.0

width = obj.width;
height = obj.height;
nbins = obj.nbins;

% process the arguments
if nargin == 1
    rect = [1, 1, width, height];
elseif nargin == 2 || nargin == 3
    if isempty(rect)
        rect = [1, 1, width, height];
    end
else
    error('Invalid number of arguments.');
end

% init
nRect = size(rect, 1);
integralHistogram = obj.integralHistogram;

histogram = zeros([nRect, nbins]);

for iBin = 1 : nbins
    histogram(:, iBin) = ...
        getregion(integralHistogram{iBin}, rect);
end

if nargin == 3
    if isNormalized == 1
        histogram = makeStochastic(histogram);
    end
end
