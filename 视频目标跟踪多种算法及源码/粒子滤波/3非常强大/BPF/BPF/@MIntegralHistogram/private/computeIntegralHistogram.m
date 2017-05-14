function obj = computeIntegralHistogram(obj)
% COMPUTEINTEGRALHISTOGRAM  Compute the integral histogram
%   obj = computeIntegralHistogram(obj) computes the integral histogram.
%
%   See also MINTEGRALHISTOGRAM, GET, SET

% Copyright 2006 Wei-Lwun Lu
% computeIntegralHistogram.m version 1.0

% get properties of the HOG descriptor
indexImage  = obj.indexImage;
weightImage = obj.weightImage;
nbins  = obj.nbins;
height = obj.height;
width  = obj.width;


% compute the integral image
[X1, Y1] = meshgrid(1:width, 1:height);
indexImage = reshape(indexImage, [], 1);
posImage = [reshape(Y1, [], 1), reshape(X1, [], 1)];
weightImage = reshape(weightImage, [], 1);

intobj = MIntegralImage;
obj.integralHistogram = cell([1, nbins]);
for iBin = 1 : nbins
    index  = find(indexImage == iBin);
    pos    = posImage(index, :);
    weight = weightImage(index);
    histogramImage = accumarray(pos, weight, [height, width]);
    
	obj.integralHistogram{iBin} = set(intobj, 'data', histogramImage);
    % obj.integralHistogram{iBin} = MIntegralImage(histogramImage);
end
