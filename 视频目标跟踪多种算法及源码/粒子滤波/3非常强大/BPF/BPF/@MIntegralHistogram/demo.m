function demo(obj, nWindows)
% DEMO          Demo the @MIntegralHistogram object
%   demo(obj)
%
%   Example:
%       demo(MIntegralHistogram);
%
%   See also MINTEGRALHISTOGRAM

% Copyright 2006 Wei-Lwun Lu
% demo.m version 1.0

load(which('MIntegralHistogram/private/testMat.mat'));
if nargin == 1
    nWindows   = 10000;
end

% generate bounding windows
rect = zeros([nWindows, 4]);
x = ceil(rand([nWindows, 2]) * 100);
y = ceil(rand([nWindows, 2]) * 100);
for iWindows = 1 : nWindows
    rect(iWindows, 1) = min(x(iWindows, :));
    rect(iWindows, 3) = max(x(iWindows, :));
    rect(iWindows, 2) = min(y(iWindows, :));
    rect(iWindows, 4) = max(y(iWindows, :));
end
fprintf('number of bounding windows: %d\n', nWindows);

% brute force
t = cputime;
bruteForce = zeros([nWindows, 10]);
for iWindows = 1 : nWindows
    subregion = indexImage(rect(iWindows, 2):rect(iWindows, 4), ...
                           rect(iWindows, 1):rect(iWindows, 3));
    bruteForce(iWindows, :) = hist(reshape(subregion, 1, []), 1:10);
end
bfTime = cputime - t;
fprintf('BruteForce: %f\n', bfTime);

% integral histogram
t = cputime;
obj = MIntegralHistogram(indexImage);
    
intImage = gethistogram(obj, rect);

intImgTime = cputime - t;
fprintf('Integral Image: %f\n', intImgTime);
fprintf('Integral Image is %f times faster\n', bfTime / intImgTime);

if any(bruteForce ~= intImage)
    error('Incorrect answer.');
end
