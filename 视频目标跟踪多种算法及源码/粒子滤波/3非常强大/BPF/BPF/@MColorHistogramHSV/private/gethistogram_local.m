function [hshist, vhist] = gethistogram_local(obj, varargin)

switch nargin
    case 1
        rect = [1, 1, obj.width, obj.height];
        isNormalized = 0;
    case 2
        rect = varargin{1};
        isNormalized = 0;
    case 3
        rect = varargin{1};
        isNormalized = varargin{2};
    otherwise
        error('Incorrect number of arguments');
end

if isempty(rect)
    rect = [1, 1, obj.width, obj.height];
end

width = obj.width;
height = obj.height;
hsbins = obj.hsbins;
hbins  = obj.hbins;
sbins  = obj.sbins;
vbins  = obj.vbins;

xmin  = max(min(rect(:, 1)), 1);
xmax  = min(max(rect(:, 3)), width);
ymin  = max(min(rect(:, 2)), 1);
ymax  = min(max(rect(:, 4)), height);

rect(:, 1) = rect(:, 1) - (xmin - 1);
rect(:, 3) = rect(:, 3) - (xmin - 1);
rect(:, 2) = rect(:, 2) - (ymin - 1);
rect(:, 4) = rect(:, 4) - (ymin - 1);

img = obj.data(ymin:ymax, xmin:xmax, :);
img = rgb2hsv(img);
width   = size(img, 2);
height  = size(img, 1);

% compute the local integral histogram
[hsImage, vImage] = computeIndexImage(img, hbins, sbins, vbins);
    
hsIntegralHistogram = MIntegralHistogram(hsImage, ones([height, width]), hsbins);
vIntegralHistogram  = MIntegralHistogram(vImage, ones([height, width]), vbins);

% get histogram
hshist = gethistogram(hsIntegralHistogram, rect, isNormalized);
vhist  = gethistogram(vIntegralHistogram, rect, isNormalized);
