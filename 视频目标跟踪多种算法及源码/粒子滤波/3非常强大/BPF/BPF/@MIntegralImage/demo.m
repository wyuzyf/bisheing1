function demo(obj, nWindows)
% DEMO          Demo the @MIntegralImage object
%   demo(obj)
%
%   Example:
%       demo(MIntegralImage);
%
%   See also MINTEGRALIMAGE

% Copyright 2006 Wei-Lwun Lu
% demo.m version 1.0

testMatrix = rand([480, 640]);
if nargin == 1
    nWindows   = 3000;
end

% generate bounding windows
rect = zeros([nWindows, 4]);
for iWindows = 1 : nWindows
    x = ceil(rand(1, 2) * 640);
    y = ceil(rand(1, 2) * 480);
    rect(iWindows, 1) = min(x);
    rect(iWindows, 3) = max(x);
    rect(iWindows, 2) = min(y);
    rect(iWindows, 4) = max(y);
end
fprintf('number of bounding windows: %d\n', nWindows);

% brute force
t = cputime;
bruteForce = zeros([1, nWindows]);
for iWindows = 1 : nWindows
    bruteForce(iWindows) = ...
        sum(sum(testMatrix(rect(iWindows, 2):rect(iWindows, 4), ...
                           rect(iWindows, 1):rect(iWindows, 3))));
end
bfTime = cputime - t;
fprintf('BruteForce: %f\n', bfTime);

% integral image
t = cputime;
intImage = zeros([1, nWindows]);
obj = MIntegralImage(testMatrix);

intImage = getregion(obj, rect);

intImgTime = cputime - t;
fprintf('Integral Image: %f\n', intImgTime);
fprintf('Integral Image is %f times faster\n', bfTime / intImgTime);

if any(~approxeq(bruteForce, intImage))
    error('Incorrect answer.');
end
